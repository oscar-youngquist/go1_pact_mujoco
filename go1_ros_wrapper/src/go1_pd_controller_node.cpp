#include <algorithm>
#include <array>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

namespace
{
constexpr size_t kNumJoints = 12;
constexpr size_t kLowCommandSize = kNumJoints * 5;

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
}  // namespace

class Go1PdControllerNode final : public rclcpp::Node
{
public:
  Go1PdControllerNode()
  : Node("go1_pd_controller")
  {
    const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 100.0);
    stand_kp_ = declare_parameter<double>("stand_kp", 60.0);
    stand_kd_ = declare_parameter<double>("stand_kd", 5.0);
    damping_kd_ = declare_parameter<double>("damping_kd", 3.0);
    transition_time_s_ = declare_parameter<double>("transition_time_s", 2.0);
    mode_ = declare_parameter<std::string>("initial_mode", "damping");

    stand_pos_ = arrayFromParameter(
        "stand_pos",
        {-0.1, 0.8, -1.5,
          0.1, 0.8, -1.5,
         -0.1, 0.8, -1.5,
          0.1, 0.8, -1.5});

    sit_pos_ = arrayFromParameter(
        "sit_pos",
        {-0.0, 1.4, -2.7,
          0.0, 1.4, -2.7,
         -0.0, 1.4, -2.7,
          0.0, 1.4, -2.7});

    low_cmd_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("low_cmd", 10);

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          handleJointState(*msg);
        });

    gamepad_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        "gamepad", 10,
        [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
          handleGamepad(*msg);
        });

    mode_sub_ = create_subscription<std_msgs::msg::String>(
        "controller_mode", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
          setMode(msg->data);
        });

    publish_timer_ = create_wall_timer(
        periodFromHz(publish_rate_hz, 100.0),
        [this]() {
          publishCommand();
        });

    RCLCPP_INFO(
        get_logger(),
        "PD controller ready. Modes: damping, sit, stand. Publishing low_cmd as q[12], dq[12], kp[12], kd[12], tau[12].");
  }

private:
  std::array<double, kNumJoints> arrayFromParameter(
      const std::string & name,
      const std::array<double, kNumJoints> & fallback)
  {
    const std::vector<double> values = declare_parameter<std::vector<double>>(
        name, std::vector<double>(fallback.begin(), fallback.end()));
    if (values.size() != kNumJoints) {
      RCLCPP_WARN(
          get_logger(), "Parameter %s has %zu values; expected %zu. Using default.",
          name.c_str(), values.size(), kNumJoints);
      return fallback;
    }

    std::array<double, kNumJoints> result{};
    std::copy(values.begin(), values.end(), result.begin());
    return result;
  }

  void handleJointState(const sensor_msgs::msg::JointState & msg)
  {
    std::unordered_map<std::string, size_t> index_by_name;
    for (size_t i = 0; i < msg.name.size(); ++i) {
      index_by_name[msg.name[i]] = i;
    }

    std::array<double, kNumJoints> next_pos{};
    std::array<double, kNumJoints> next_vel{};
    for (size_t i = 0; i < kNumJoints; ++i) {
      const auto it = index_by_name.find(kJointNames[i]);
      if (it == index_by_name.end() || it->second >= msg.position.size()) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "JointState missing expected joint %s; not publishing controller commands.",
            kJointNames[i]);
        have_state_ = false;
        return;
      }

      next_pos[i] = msg.position[it->second];
      if (it->second < msg.velocity.size()) {
        next_vel[i] = msg.velocity[it->second];
      }
    }

    current_pos_ = next_pos;
    current_vel_ = next_vel;
    if (!have_state_) {
      command_start_pos_ = current_pos_;
      mode_start_time_ = now();
    }
    have_state_ = true;
  }

  void handleGamepad(const std_msgs::msg::Float32MultiArray & msg)
  {
    if (msg.data.size() < 18) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Ignoring gamepad message with %zu values; expected at least 18.", msg.data.size());
      return;
    }

    const bool A = msg.data[4] != 0.0f;
    const bool B = msg.data[5] != 0.0f;
    const bool LB = msg.data[8] != 0.0f;
    const bool RB = msg.data[9] != 0.0f;
    const bool START = msg.data[11] != 0.0f;
    const bool RT = msg.data[13] != 0.0f;

    if (rising(START, prev_start_) || (LB && rising(RT, prev_rt_))) {
      setMode("stand");
    }
    if (LB && rising(RB, prev_rb_)) {
      setMode("sit");
    }
    if (LB && rising(B, prev_b_)) {
      setMode("damping");
    }
    if (LB && rising(A, prev_a_)) {
      setMode("stand");
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "LB+A maps to stand in the PD controller. Walking requires the future PACT ROS controller node.");
    }

    prev_a_ = A;
    prev_b_ = B;
    prev_rb_ = RB;
    prev_rt_ = RT;
    prev_start_ = START;
  }

  bool rising(bool current, bool previous) const
  {
    return current && !previous;
  }
  void setMode(const std::string & requested_mode)
  {
    if (requested_mode != "damping" && requested_mode != "sit" && requested_mode != "stand") {
      RCLCPP_WARN(get_logger(), "Ignoring unknown controller mode: %s", requested_mode.c_str());
      return;
    }

    command_start_pos_ = current_pos_;
    mode_start_time_ = now();
    mode_ = requested_mode;
    RCLCPP_INFO(get_logger(), "Controller mode: %s", mode_.c_str());
  }

  std::array<double, kNumJoints> targetForMode() const
  {
    if (mode_ == "stand") {
      return stand_pos_;
    }
    if (mode_ == "sit") {
      return sit_pos_;
    }
    return current_pos_;
  }

  double transitionAlpha() const
  {
    if (transition_time_s_ <= 0.0) {
      return 1.0;
    }
    return std::clamp((now() - mode_start_time_).seconds() / transition_time_s_, 0.0, 1.0);
  }

  void publishCommand()
  {
    if (!have_state_) {
      return;
    }

    const auto target = targetForMode();
    const double alpha = mode_ == "damping" ? 1.0 : transitionAlpha();

    std_msgs::msg::Float32MultiArray msg;
    msg.data.assign(kLowCommandSize, 0.0f);

    for (size_t i = 0; i < kNumJoints; ++i) {
      const double q_des = command_start_pos_[i] + alpha * (target[i] - command_start_pos_[i]);
      msg.data[i] = static_cast<float>(q_des);
      msg.data[12 + i] = 0.0f;
      msg.data[24 + i] = static_cast<float>(mode_ == "damping" ? 0.0 : stand_kp_);
      msg.data[36 + i] = static_cast<float>(mode_ == "damping" ? damping_kd_ : stand_kd_);
      msg.data[48 + i] = 0.0f;
    }

    low_cmd_pub_->publish(msg);
  }

  std::string mode_ = "damping";
  double stand_kp_ = 60.0;
  double stand_kd_ = 5.0;
  double damping_kd_ = 3.0;
  double transition_time_s_ = 2.0;
  bool have_state_ = false;

  std::array<double, kNumJoints> current_pos_{};
  std::array<double, kNumJoints> current_vel_{};
  std::array<double, kNumJoints> command_start_pos_{};
  std::array<double, kNumJoints> stand_pos_{};
  std::array<double, kNumJoints> sit_pos_{};
  rclcpp::Time mode_start_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr low_cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr gamepad_sub_;
  bool prev_a_ = false;
  bool prev_b_ = false;
  bool prev_rb_ = false;
  bool prev_rt_ = false;
  bool prev_start_ = false;

  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go1PdControllerNode>());
  rclcpp::shutdown();
  return 0;
}