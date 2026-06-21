#include <array>
#include <cerrno>
#include <cmath>
#include <chrono>
#include <cstring>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

using namespace std::chrono_literals;

namespace
{
constexpr size_t kGamepadValueCount = 18;

std::chrono::nanoseconds periodFromHz(double hz, double fallback_hz)
{
  if (hz <= 0.0 || !std::isfinite(hz)) {
    hz = fallback_hz;
  }
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / hz));
}
}  // namespace

class Go1UdpGamepadNode final : public rclcpp::Node
{
public:
  Go1UdpGamepadNode()
  : Node("go1_udp_gamepad")
  {
    port_ = declare_parameter<int>("port", 26760);
    const double poll_rate_hz = declare_parameter<double>("poll_rate_hz", 200.0);
    const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 100.0);

    gamepad_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("gamepad", 10);
    openSocket();

    poll_timer_ = create_wall_timer(
        periodFromHz(poll_rate_hz, 200.0),
        [this]() {
          pollSocket();
        });

    publish_timer_ = create_wall_timer(
        periodFromHz(publish_rate_hz, 100.0),
        [this]() {
          publishGamepad();
        });

    RCLCPP_INFO(get_logger(), "Listening for Windows gamepad UDP packets on port %d", port_);
    RCLCPP_INFO(get_logger(), "Publishing /gamepad as [lx, ly, rx, ry, A, B, X, Y, LB, RB, BACK, START, LT, RT, UP, DOWN, LEFT, RIGHT]");
  }

  ~Go1UdpGamepadNode() override
  {
    if (fd_ >= 0) {
      close(fd_);
    }
  }

private:
  void openSocket()
  {
    fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) {
      throw std::runtime_error(std::string("socket failed: ") + std::strerror(errno));
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(static_cast<uint16_t>(port_));

    if (bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      throw std::runtime_error(std::string("bind failed: ") + std::strerror(errno));
    }

    const int flags = fcntl(fd_, F_GETFL, 0);
    fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
  }

  void pollSocket()
  {
    char buffer[512];
    while (true) {
      const ssize_t bytes = recv(fd_, buffer, sizeof(buffer) - 1, 0);
      if (bytes < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
          RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "UDP receive failed: %s", std::strerror(errno));
        }
        return;
      }

      buffer[bytes] = '\0';
      parsePacket(buffer);
      last_packet_time_ = now();
    }
  }

  void parsePacket(const std::string & packet)
  {
    std::istringstream stream(packet);
    std::string magic;
    stream >> magic;
    if (magic != "GPAD1") {
      return;
    }

    std::array<float, kGamepadValueCount> values{};
    for (float & value : values) {
      if (!(stream >> value)) {
        return;
      }
    }

    latest_values_.assign(values.begin(), values.end());
  }

  void publishGamepad()
  {
    std_msgs::msg::Float32MultiArray msg;
    msg.data = latest_values_;
    gamepad_pub_->publish(msg);
  }

  int port_ = 26760;
  int fd_ = -1;
  std::vector<float> latest_values_ = std::vector<float>(kGamepadValueCount, 0.0f);
  rclcpp::Time last_packet_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr gamepad_pub_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go1UdpGamepadNode>());
  rclcpp::shutdown();
  return 0;
}