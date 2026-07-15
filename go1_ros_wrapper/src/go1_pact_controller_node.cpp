#include <algorithm>
#include <array>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include "gamepad.hpp"
#include "pact_controller.hpp"
#include "robot_interface.hpp"
#include "state_machine.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr size_t kNumJoints = 12;
constexpr size_t kLowCommandSize = kNumJoints * 5;

std::filesystem::path defaultWorkspaceDir()
{
  if (const char * env = std::getenv("HCRL_LAB_CONTROLLER_DIR")) {
    return std::filesystem::path(env);
  }
  return std::filesystem::current_path();
}

const std::array<const char *, kNumJoints> kJointNames = {
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};

std::chrono::nanoseconds periodFromHz(double hz, double fallback_hz)
{
  if (hz <= 0.0 || !std::isfinite(hz)) {
    hz = fallback_hz;
  }
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / hz));
}

std::array<float, 3> quatToRpy(float w, float x, float y, float z)
{
  const float sinr_cosp = 2.0f * (w * x + y * z);
  const float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
  const float roll = std::atan2(sinr_cosp, cosr_cosp);

  const float sinp = 2.0f * (w * y - z * x);
  const float pitch = std::fabs(sinp) >= 1.0f ? std::copysign(static_cast<float>(M_PI) / 2.0f, sinp) : std::asin(sinp);

  const float siny_cosp = 2.0f * (w * z + x * y);
  const float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
  const float yaw = std::atan2(siny_cosp, cosy_cosp);

  return {roll, pitch, yaw};
}
}  // namespace

class Go1PactControllerNode final : public rclcpp::Node
{
public:
  Go1PactControllerNode()
  : Node("go1_pact_controller")
  {
    const auto default_workspace_dir = defaultWorkspaceDir();
    const std::string default_config =
        (default_workspace_dir / "go1_pact_mujoco" / "go1_pact_deploy" / "params" / "pact_config.yaml").string();
    const std::string default_mujoco_dir =
        (default_workspace_dir / "go1_pact_mujoco").string();

    config_path_ = declare_parameter<std::string>("config_path", default_config);
    mujoco_project_dir_ = declare_parameter<std::string>("mujoco_project_dir", default_mujoco_dir);
    const double control_rate_hz = declare_parameter<double>("control_rate_hz", 100.0);
    damping_kd_ = declare_parameter<double>("damping_kd", 3.0);
    debug_commands_ = declare_parameter<bool>("debug_commands", false);
    mode_ = declare_parameter<std::string>("initial_mode", "damping");
    velocity_command_frame_id_ = declare_parameter<std::string>("velocity_command_frame_id", "base_link");

    initializeController();

    low_cmd_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("low_cmd", 10);
    velocity_command_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("velocity_command", 10);

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          handleJointState(*msg);
        });

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu", 10,
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
          handleImu(*msg);
        });

    gamepad_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        "gamepad", 10,
        [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
          handleGamepad(*msg);
        });

    mode_sub_ = create_subscription<std_msgs::msg::String>(
        "controller_mode", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
          requestMode(msg->data);
        });

    control_timer_ = create_wall_timer(
        periodFromHz(control_rate_hz, 100.0),
        [this]() {
          controlStep();
        });

    RCLCPP_INFO(get_logger(), "PACT ROS controller ready. Modes: damping, sit, stand, ctrl.");
  }

private:
  void initializeController()
  {
    ctrl_ = std::make_unique<PACTController>(config_path_);
    ctrl_->loadParam();

    const PACTRLCfg cfg(config_path_);
    if (cfg.command_limit_max_abs.size() >= 3) {
      gamepad_.setCommandLimits({
          cfg.command_limit_max_abs[0],
          cfg.command_limit_max_abs[1],
          cfg.command_limit_max_abs[2],
      });
    } else {
      RCLCPP_WARN(
          get_logger(),
          "PACT config command_limit_max_abs has fewer than 3 values; using default gamepad command limits.");
    }

    const auto old_cwd = std::filesystem::current_path();
    std::filesystem::current_path(mujoco_project_dir_);
    ctrl_->loadPolicy();
    std::filesystem::current_path(old_cwd);

    robot_interface_.kp.fill(0.0f);
    robot_interface_.kd.fill(damping_kd_);
    robot_interface_.jpos_des.fill(0.0f);
    robot_interface_.jvel_des.fill(0.0f);
    robot_interface_.tau_ff.fill(0.0f);
  }

  void handleJointState(const sensor_msgs::msg::JointState & msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);

    std::unordered_map<std::string, size_t> index_by_name;
    for (size_t i = 0; i < msg.name.size(); ++i) {
      index_by_name[msg.name[i]] = i;
    }

    for (size_t i = 0; i < kNumJoints; ++i) {
      const auto it = index_by_name.find(kJointNames[i]);
      if (it == index_by_name.end() || it->second >= msg.position.size()) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "JointState missing expected joint %s; controller output paused.", kJointNames[i]);
        have_joint_state_ = false;
        return;
      }

      state_.motorState[i].q = static_cast<float>(msg.position[it->second]);
      state_.motorState[i].dq = it->second < msg.velocity.size() ? static_cast<float>(msg.velocity[it->second]) : 0.0f;
      state_.motorState[i].tauEst = it->second < msg.effort.size() ? static_cast<float>(msg.effort[it->second]) : 0.0f;
    }

    have_joint_state_ = true;
    updateRobotInterfaceLocked();
  }

  void handleImu(const sensor_msgs::msg::Imu & msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);

    state_.imu.quaternion[0] = static_cast<float>(msg.orientation.w);
    state_.imu.quaternion[1] = static_cast<float>(msg.orientation.x);
    state_.imu.quaternion[2] = static_cast<float>(msg.orientation.y);
    state_.imu.quaternion[3] = static_cast<float>(msg.orientation.z);

    state_.imu.gyroscope[0] = static_cast<float>(msg.angular_velocity.x);
    state_.imu.gyroscope[1] = static_cast<float>(msg.angular_velocity.y);
    state_.imu.gyroscope[2] = static_cast<float>(msg.angular_velocity.z);

    state_.imu.accelerometer[0] = static_cast<float>(msg.linear_acceleration.x);
    state_.imu.accelerometer[1] = static_cast<float>(msg.linear_acceleration.y);
    state_.imu.accelerometer[2] = static_cast<float>(msg.linear_acceleration.z);

    const auto rpy = quatToRpy(
        state_.imu.quaternion[0], state_.imu.quaternion[1],
        state_.imu.quaternion[2], state_.imu.quaternion[3]);
    state_.imu.rpy[0] = rpy[0];
    state_.imu.rpy[1] = rpy[1];
    state_.imu.rpy[2] = rpy[2];

    have_imu_ = true;
    updateRobotInterfaceLocked();
  }

  void updateRobotInterfaceLocked()
  {
    if (have_joint_state_ && have_imu_) {
      robot_interface_.SetState(state_);
    }
  }

  void handleGamepad(const std_msgs::msg::Float32MultiArray & msg)
  {
    if (msg.data.size() < 18) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Ignoring gamepad message with %zu values; expected at least 18.", msg.data.size());
      return;
    }

    xRockerBtnDataStruct key_data{};
    key_data.lx = msg.data[0];
    key_data.ly = msg.data[1];
    key_data.rx = msg.data[2];
    key_data.ry = msg.data[3];
    key_data.btn.components.A = msg.data[4] != 0.0f;
    key_data.btn.components.B = msg.data[5] != 0.0f;
    key_data.btn.components.X = msg.data[6] != 0.0f;
    key_data.btn.components.Y = msg.data[7] != 0.0f;
    key_data.btn.components.L1 = msg.data[8] != 0.0f;
    key_data.btn.components.R1 = msg.data[9] != 0.0f;
    key_data.btn.components.select = msg.data[10] != 0.0f;
    key_data.btn.components.start = msg.data[11] != 0.0f;
    key_data.btn.components.L2 = msg.data[12] != 0.0f;
    key_data.btn.components.R2 = msg.data[13] != 0.0f;
    key_data.btn.components.up = msg.data[14] != 0.0f;
    key_data.btn.components.down = msg.data[15] != 0.0f;
    key_data.btn.components.left = msg.data[16] != 0.0f;
    key_data.btn.components.right = msg.data[17] != 0.0f;

    gamepad_.update(key_data);
    updateModeFromGamepad();
  }

  void updateModeFromGamepad()
  {
    if (gamepad_.R2.on_press && gamepad_.L1.pressed) {
      requestMode("stand");
    }
    if (gamepad_.R1.on_press && gamepad_.L1.pressed) {
      requestMode("sit");
    }
    if (gamepad_.A.on_press && gamepad_.L1.pressed) {
      requestMode("ctrl");
    }
    if (gamepad_.B.on_press && gamepad_.L1.pressed) {
      requestMode("damping");
    }
    if (gamepad_.start.on_press) {
      requestMode("stand");
    }
  }

  void requestMode(const std::string & requested_mode)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    setModeLocked(requested_mode);
  }

  void setModeLocked(const std::string & requested_mode)
  {
    if (requested_mode != "damping" && requested_mode != "sit" &&
        requested_mode != "stand" && requested_mode != "ctrl") {
      RCLCPP_WARN(get_logger(), "Ignoring unknown PACT controller mode: %s", requested_mode.c_str());
      return;
    }

    if (!have_joint_state_) {
      mode_ = requested_mode;
      RCLCPP_WARN(get_logger(), "Mode set to %s before joint state is available.", mode_.c_str());
      return;
    }

    if (requested_mode == "damping") {
      mode_ = requested_mode;
      state_machine_.Stop();
      dampingLocked();
    } else if (requested_mode == "sit") {
      mode_ = requested_mode;
      if (state_machine_.Sit()) {
        ctrl_->save_jpos(robot_interface_);
        robot_interface_.jpos_des = ctrl_->start_pos;
        robot_interface_.kp.fill(ctrl_->stand_kp);
        robot_interface_.kd.fill(ctrl_->stand_kd);
        robot_interface_.jvel_des.fill(0.0f);
        robot_interface_.tau_ff.fill(0.0f);
      }
    } else if (requested_mode == "stand") {
      mode_ = requested_mode;
      if (state_machine_.Stand()) {
        ctrl_->save_jpos(robot_interface_);
        robot_interface_.jpos_des = ctrl_->start_pos;
        robot_interface_.kp.fill(ctrl_->stand_kp);
        robot_interface_.kd.fill(ctrl_->stand_kd);
        robot_interface_.jvel_des.fill(0.0f);
        robot_interface_.tau_ff.fill(0.0f);
      }
    } else if (requested_mode == "ctrl") {
      mode_ = requested_mode;
      state_machine_.Ctrl();
      ctrl_->reset(robot_interface_, gamepad_);
      robot_interface_.jpos_des = ctrl_->stand_pos;
      robot_interface_.kp.fill(ctrl_->ctrl_kp);
      robot_interface_.kd.fill(ctrl_->ctrl_kd);
      robot_interface_.jvel_des.fill(0.0f);
      robot_interface_.tau_ff.fill(0.0f);
    }

    RCLCPP_INFO(get_logger(), "PACT controller mode: %s", mode_.c_str());
  }

  void controlStep()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!have_joint_state_ || !have_imu_) {
      return;
    }

    if (!mode_initialized_) {
      const std::string initial_mode = mode_;
      setModeLocked(initial_mode);
      mode_initialized_ = true;
    }

    ctrl_->GetInput(robot_interface_, gamepad_);
    publishVelocityCommand();

    if (mode_ == "sit") {
      state_machine_.Sitting(*ctrl_);
      robot_interface_.jpos_des = ctrl_->jpos_des;
    } else if (mode_ == "stand") {
      state_machine_.Standing(*ctrl_);
      robot_interface_.jpos_des = ctrl_->jpos_des;
    } else if (mode_ == "ctrl") {
      ctrl_->Calculate();
      robot_interface_.jpos_des = ctrl_->jpos_des;
      robot_interface_.tau_ff = ctrl_->jtau_des;
      robot_interface_.jvel_des.fill(0.0f);
    } else {
      dampingLocked();
    }

    if (debug_commands_ && ((debug_counter_++ % 50) == 0)) {
      RCLCPP_INFO(get_logger(), "mode=%s gp(lx=%.3f ly=%.3f rx=%.3f) qdes0=%.3f kp0=%.1f kd0=%.1f tau0=%.3f", mode_.c_str(), gamepad_.lx, gamepad_.ly, gamepad_.rx, robot_interface_.jpos_des[0], robot_interface_.kp[0], robot_interface_.kd[0], robot_interface_.tau_ff[0]);
    }

    publishLowCommand();
  }

  void dampingLocked()
  {
    robot_interface_.jpos_des.fill(0.0f);
    robot_interface_.jvel_des.fill(0.0f);
    robot_interface_.kp.fill(0.0f);
    robot_interface_.kd.fill(damping_kd_);
    robot_interface_.tau_ff.fill(0.0f);
  }

  void publishLowCommand()
  {
    std_msgs::msg::Float32MultiArray msg;
    msg.data.assign(kLowCommandSize, 0.0f);

    for (size_t i = 0; i < kNumJoints; ++i) {
      msg.data[i] = robot_interface_.jpos_des[i];
      msg.data[12 + i] = robot_interface_.jvel_des[i];
      msg.data[24 + i] = robot_interface_.kp[i];
      msg.data[36 + i] = robot_interface_.kd[i];
      msg.data[48 + i] = robot_interface_.tau_ff[i];
    }

    low_cmd_pub_->publish(msg);
  }

  void publishVelocityCommand()
  {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = velocity_command_frame_id_;
    msg.twist.linear.x = ctrl_->cmd[0];
    msg.twist.linear.y = ctrl_->cmd[1];
    msg.twist.linear.z = 0.0;
    msg.twist.angular.x = 0.0;
    msg.twist.angular.y = 0.0;
    msg.twist.angular.z = ctrl_->cmd[2];
    velocity_command_pub_->publish(msg);
  }

  std::string config_path_;
  std::string mujoco_project_dir_;
  std::string mode_ = "damping";
  std::string velocity_command_frame_id_ = "base_link";
  float damping_kd_ = 3.0f;
  bool debug_commands_ = false;
  size_t debug_counter_ = 0;

  std::mutex state_mutex_;
  bool have_joint_state_ = false;
  bool have_imu_ = false;
  bool mode_initialized_ = false;
  LowState state_{};
  BasicRobotInterface robot_interface_;
  Gamepad gamepad_;
  PACTStateMachine state_machine_;
  std::unique_ptr<PACTController> ctrl_;

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr low_cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_command_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr gamepad_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go1PactControllerNode>());
  rclcpp::shutdown();
  return 0;
}
