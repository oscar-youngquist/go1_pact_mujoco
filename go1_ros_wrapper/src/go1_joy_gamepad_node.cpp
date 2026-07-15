#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

namespace
{
constexpr size_t kGamepadValueCount = 18;

float axisValue(const sensor_msgs::msg::Joy & msg, int index, bool invert)
{
  if (index < 0 || static_cast<size_t>(index) >= msg.axes.size()) {
    return 0.0f;
  }
  const float value = static_cast<float>(msg.axes[static_cast<size_t>(index)]);
  return invert ? -value : value;
}

float buttonValue(const sensor_msgs::msg::Joy & msg, int index)
{
  if (index < 0 || static_cast<size_t>(index) >= msg.buttons.size()) {
    return 0.0f;
  }
  return msg.buttons[static_cast<size_t>(index)] != 0 ? 1.0f : 0.0f;
}
}  // namespace

class Go1JoyGamepadNode final : public rclcpp::Node
{
public:
  Go1JoyGamepadNode()
  : Node("go1_joy_gamepad")
  {
    axis_lx_ = declare_parameter<int>("axis_lx", 0);
    axis_ly_ = declare_parameter<int>("axis_ly", 1);
    axis_rx_ = declare_parameter<int>("axis_rx", 2);
    axis_ry_ = declare_parameter<int>("axis_ry", 3);
    invert_lx_ = declare_parameter<bool>("invert_lx", false);
    invert_ly_ = declare_parameter<bool>("invert_ly", false);
    invert_rx_ = declare_parameter<bool>("invert_rx", false);
    invert_ry_ = declare_parameter<bool>("invert_ry", false);

    button_x_ = declare_parameter<int>("button_x", 0);
    button_a_ = declare_parameter<int>("button_a", 1);
    button_b_ = declare_parameter<int>("button_b", 2);
    button_y_ = declare_parameter<int>("button_y", 3);
    button_lb_ = declare_parameter<int>("button_lb", 4);
    button_rb_ = declare_parameter<int>("button_rb", 5);
    button_lt_ = declare_parameter<int>("button_lt", 6);
    button_rt_ = declare_parameter<int>("button_rt", 7);
    button_back_ = declare_parameter<int>("button_back", 8);
    button_start_ = declare_parameter<int>("button_start", 9);

    dpad_axis_x_ = declare_parameter<int>("dpad_axis_x", 4);
    dpad_axis_y_ = declare_parameter<int>("dpad_axis_y", 5);
    dpad_threshold_ = static_cast<float>(declare_parameter<double>("dpad_threshold", 0.5));

    gamepad_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("gamepad", 10);
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy",
      10,
      [this](const sensor_msgs::msg::Joy::SharedPtr msg) {
        handleJoy(*msg);
      });

    RCLCPP_INFO(
      get_logger(),
      "Mapping /joy to Go1 gamepad order [lx, ly, rx, ry, A, B, X, Y, LB, RB, BACK, START, LT, RT, UP, DOWN, LEFT, RIGHT]");
  }

private:
  void handleJoy(const sensor_msgs::msg::Joy & msg)
  {
    std_msgs::msg::Float32MultiArray out;
    out.data.assign(kGamepadValueCount, 0.0f);

    out.data[0] = axisValue(msg, axis_lx_, invert_lx_);
    out.data[1] = axisValue(msg, axis_ly_, invert_ly_);
    out.data[2] = axisValue(msg, axis_rx_, invert_rx_);
    out.data[3] = axisValue(msg, axis_ry_, invert_ry_);

    out.data[4] = buttonValue(msg, button_a_);
    out.data[5] = buttonValue(msg, button_b_);
    out.data[6] = buttonValue(msg, button_x_);
    out.data[7] = buttonValue(msg, button_y_);
    out.data[8] = buttonValue(msg, button_lb_);
    out.data[9] = buttonValue(msg, button_rb_);
    out.data[10] = buttonValue(msg, button_back_);
    out.data[11] = buttonValue(msg, button_start_);
    out.data[12] = buttonValue(msg, button_lt_);
    out.data[13] = buttonValue(msg, button_rt_);

    const float dpad_x = axisValue(msg, dpad_axis_x_, false);
    const float dpad_y = axisValue(msg, dpad_axis_y_, false);
    out.data[14] = dpad_y > dpad_threshold_ ? 1.0f : 0.0f;
    out.data[15] = dpad_y < -dpad_threshold_ ? 1.0f : 0.0f;
    out.data[16] = dpad_x > dpad_threshold_ ? 1.0f : 0.0f;
    out.data[17] = dpad_x < -dpad_threshold_ ? 1.0f : 0.0f;

    gamepad_pub_->publish(out);
  }

  int axis_lx_ = 0;
  int axis_ly_ = 1;
  int axis_rx_ = 2;
  int axis_ry_ = 3;
  bool invert_lx_ = false;
  bool invert_ly_ = false;
  bool invert_rx_ = false;
  bool invert_ry_ = false;

  int button_x_ = 0;
  int button_a_ = 1;
  int button_b_ = 2;
  int button_y_ = 3;
  int button_lb_ = 4;
  int button_rb_ = 5;
  int button_lt_ = 6;
  int button_rt_ = 7;
  int button_back_ = 8;
  int button_start_ = 9;

  int dpad_axis_x_ = 4;
  int dpad_axis_y_ = 5;
  float dpad_threshold_ = 0.5f;

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr gamepad_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go1JoyGamepadNode>());
  rclcpp::shutdown();
  return 0;
}
