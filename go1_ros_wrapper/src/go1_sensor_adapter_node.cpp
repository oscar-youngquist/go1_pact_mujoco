#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

class Go1SensorAdapterNode final : public rclcpp::Node
{
public:
  Go1SensorAdapterNode()
  : Node("go1_sensor_adapter")
  {
    const bool enable_camera = declare_parameter<bool>("enable_camera", true);
    const bool enable_depth = declare_parameter<bool>("enable_depth", true);
    const bool enable_gps = declare_parameter<bool>("enable_gps", true);

    if (enable_camera) {
      color_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
        "camera/color/image_raw", rclcpp::SensorDataQoS());
      color_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        "camera/color/camera_info", rclcpp::SensorDataQoS());

      color_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        declare_parameter<std::string>(
          "color_image_input", "/go1/sensors/realsense/d435i/color/image_raw"),
        rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
          color_image_pub_->publish(*message);
        });
      color_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        declare_parameter<std::string>(
          "color_info_input", "/go1/sensors/realsense/d435i/color/camera_info"),
        rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
          color_info_pub_->publish(*message);
        });
    }

    if (enable_camera && enable_depth) {
      depth_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
        "camera/depth/image_raw", rclcpp::SensorDataQoS());
      depth_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        "camera/depth/camera_info", rclcpp::SensorDataQoS());

      depth_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        declare_parameter<std::string>(
          "depth_image_input", "/go1/sensors/realsense/d435i/depth/image_rect_raw"),
        rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
          depth_image_pub_->publish(*message);
        });
      depth_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        declare_parameter<std::string>(
          "depth_info_input", "/go1/sensors/realsense/d435i/depth/camera_info"),
        rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
          depth_info_pub_->publish(*message);
        });
    }

    if (enable_gps) {
      gps_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(
        "gps/fix", rclcpp::SensorDataQoS());
      gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        declare_parameter<std::string>(
          "gps_input", "/go1/sensors/ellipse/imu/nav_sat_fix"),
        rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr message) {
          gps_pub_->publish(*message);
        });
    }

    RCLCPP_INFO(
      get_logger(), "Physical sensor adapter started (camera=%s, depth=%s, gps=%s)",
      enable_camera ? "on" : "off",
      enable_camera && enable_depth ? "on" : "off",
      enable_gps ? "on" : "off");
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go1SensorAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
