#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <unitree_legged_sdk/unitree_legged_sdk.h>

using namespace std::chrono_literals;
using namespace UNITREE_LEGGED_SDK;

namespace
{
constexpr size_t kNumJoints = 12;
constexpr size_t kLowCommandSize = kNumJoints * 5;

const std::array<std::string, kNumJoints> kJointNames = {
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};

template <typename MsgArrayT, typename UnitreeArrayT>
void copy3(MsgArrayT & dst, const UnitreeArrayT & src)
{
  dst.x = src[0];
  dst.y = src[1];
  dst.z = src[2];
}
}  // namespace

class Go1UnitreeBridgeNode final : public rclcpp::Node
{
public:
  Go1UnitreeBridgeNode()
  : Node("go1_unitree_bridge"),
    safe_(LeggedType::Go1)
  {
    const auto robot_ip = declare_parameter<std::string>("robot_ip", "192.168.123.10");
    const int local_port = declare_parameter<int>("local_port", 8090);
    const int robot_port = declare_parameter<int>("robot_port", 8007);
    const double udp_rate_hz = declare_parameter<double>("udp_rate_hz", 500.0);
    const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 100.0);

    command_timeout_s_ = declare_parameter<double>("command_timeout_s", 0.2);
    power_protect_level_ = declare_parameter<int>("power_protect_level", 9);
    send_damping_on_timeout_ = declare_parameter<bool>("send_damping_on_timeout", true);
    timeout_damping_kd_ = declare_parameter<double>("timeout_damping_kd", 3.0);
    send_commands_ = declare_parameter<bool>("send_commands", false);

    udp_ = std::make_unique<UDP>(LOWLEVEL, local_port, robot_ip.c_str(), robot_port);
    udp_->InitCmdData(low_cmd_);

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    low_cmd_echo_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("low_cmd_echo", 10);

    low_cmd_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        "low_cmd", 10,
        [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
          handleLowCommand(*msg);
        });

    recv_timer_ = create_wall_timer(
        periodFromHz(udp_rate_hz),
        [this]() {
          receiveState();
        });

    send_timer_ = create_wall_timer(
        periodFromHz(udp_rate_hz),
        [this]() {
          sendCommand();
        });

    publish_timer_ = create_wall_timer(
        periodFromHz(publish_rate_hz),
        [this]() {
          publishState();
        });

    if (send_commands_) {
      RCLCPP_WARN(
          get_logger(),
          "Go1 Unitree bridge ready and MOTOR COMMAND SENDING IS ENABLED. "
          "Command topic expects 60 floats: q[12], dq[12], kp[12], kd[12], tau[12].");
    } else {
      RCLCPP_WARN(
          get_logger(),
          "Go1 Unitree bridge ready in observe-only mode. It will receive/publish robot state, "
          "but will not send motor commands until send_commands:=true.");
    }
  }

private:
  static std::chrono::nanoseconds periodFromHz(double hz)
  {
    if (hz <= 0.0 || !std::isfinite(hz)) {
      hz = 100.0;
    }
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / hz));
  }

  void handleLowCommand(const std_msgs::msg::Float32MultiArray & msg)
  {
    if (msg.data.size() != kLowCommandSize) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Ignoring low_cmd with %zu values; expected %zu.",
          msg.data.size(), kLowCommandSize);
      return;
    }

    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      for (size_t i = 0; i < kNumJoints; ++i) {
        low_cmd_.motorCmd[i].q = msg.data[i];
        low_cmd_.motorCmd[i].dq = msg.data[12 + i];
        low_cmd_.motorCmd[i].Kp = msg.data[24 + i];
        low_cmd_.motorCmd[i].Kd = msg.data[36 + i];
        low_cmd_.motorCmd[i].tau = msg.data[48 + i];
      }
      last_command_time_ = now();
      have_command_ = true;
    }

    low_cmd_echo_pub_->publish(msg);
  }

  void receiveState()
  {
    udp_->Recv();
    std::lock_guard<std::mutex> lock(state_mutex_);
    udp_->GetRecv(low_state_);
  }

  void sendCommand()
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);

    if (!have_command_) {
      return;
    }

    if (!send_commands_) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Received low_cmd, but send_commands is false. Not sending commands to the robot.");
      return;
    }

    if (commandTimedOut()) {
      applyTimeoutCommand();
    }

    safe_.PositionLimit(low_cmd_);
    const int protect_result = safe_.PowerProtect(low_cmd_, low_state_, power_protect_level_);
    if (protect_result < 0) {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Unitree PowerProtect rejected the outgoing command.");
      applyTimeoutCommand();
    }

    udp_->SetSend(low_cmd_);
    udp_->Send();
  }

  bool commandTimedOut() const
  {
    return (now() - last_command_time_).seconds() > command_timeout_s_;
  }

  void applyTimeoutCommand()
  {
    for (size_t i = 0; i < kNumJoints; ++i) {
      low_cmd_.motorCmd[i].q = 0.0f;
      low_cmd_.motorCmd[i].dq = 0.0f;
      low_cmd_.motorCmd[i].Kp = 0.0f;
      low_cmd_.motorCmd[i].Kd = send_damping_on_timeout_ ? static_cast<float>(timeout_damping_kd_) : 0.0f;
      low_cmd_.motorCmd[i].tau = 0.0f;
    }
  }

  void publishState()
  {
    LowState state_snapshot;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      state_snapshot = low_state_;
    }

    const auto stamp = now();

    sensor_msgs::msg::JointState joint_msg;
    joint_msg.header.stamp = stamp;
    joint_msg.name.assign(kJointNames.begin(), kJointNames.end());
    joint_msg.position.resize(kNumJoints);
    joint_msg.velocity.resize(kNumJoints);
    joint_msg.effort.resize(kNumJoints);

    for (size_t i = 0; i < kNumJoints; ++i) {
      const auto & motor = state_snapshot.motorState[i];
      joint_msg.position[i] = motor.q;
      joint_msg.velocity[i] = motor.dq;
      joint_msg.effort[i] = motor.tauEst;
    }
    joint_state_pub_->publish(joint_msg);

    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = stamp;
    imu_msg.header.frame_id = "go1_imu";
    imu_msg.orientation.w = state_snapshot.imu.quaternion[0];
    imu_msg.orientation.x = state_snapshot.imu.quaternion[1];
    imu_msg.orientation.y = state_snapshot.imu.quaternion[2];
    imu_msg.orientation.z = state_snapshot.imu.quaternion[3];
    copy3(imu_msg.angular_velocity, state_snapshot.imu.gyroscope);
    copy3(imu_msg.linear_acceleration, state_snapshot.imu.accelerometer);
    imu_pub_->publish(imu_msg);
  }

  Safety safe_;
  std::unique_ptr<UDP> udp_;
  LowCmd low_cmd_{};
  LowState low_state_{};

  std::mutex state_mutex_;
  std::mutex cmd_mutex_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr low_cmd_echo_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr low_cmd_sub_;

  rclcpp::TimerBase::SharedPtr recv_timer_;
  rclcpp::TimerBase::SharedPtr send_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  bool have_command_ = false;
  double command_timeout_s_ = 0.2;
  int power_protect_level_ = 9;
  bool send_damping_on_timeout_ = true;
  double timeout_damping_kd_ = 3.0;
  bool send_commands_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go1UnitreeBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
