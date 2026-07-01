#include <algorithm>
#include <atomic>
#include <array>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <cmath>
#include <memory>
#include <mutex>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

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

std::chrono::nanoseconds periodFromHz(double hz, double fallback_hz)
{
  if (hz <= 0.0 || !std::isfinite(hz)) {
    hz = fallback_hz;
  }
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / hz));
}
}  // namespace

class Go1MujocoSimNode final : public rclcpp::Node
{
public:
  Go1MujocoSimNode()
  : Node("go1_mujoco_sim")
  {
    const std::string default_model =
        (defaultWorkspaceDir() / "go1_pact_mujoco" / "Resources" / "scene.xml").string();
    model_path_ = declare_parameter<std::string>("model_path", default_model);
    const double sim_rate_hz = declare_parameter<double>("sim_rate_hz", 500.0);
    const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 100.0);
    command_timeout_s_ = declare_parameter<double>("command_timeout_s", 0.2);
    publish_when_paused_ = declare_parameter<bool>("publish_when_paused", true);
    enable_viewer_ = declare_parameter<bool>("enable_viewer", false);
    render_period_ = periodFromHz(declare_parameter<double>("render_rate_hz", 30.0), 30.0);
    viewer_width_ = declare_parameter<int>("viewer_width", 1200);
    viewer_height_ = declare_parameter<int>("viewer_height", 900);
    enable_sim_camera_ = declare_parameter<bool>("enable_sim_camera", true);
    camera_name_ = declare_parameter<std::string>("camera_name", "front_camera");
    camera_rate_hz_ = declare_parameter<double>("camera_rate_hz", 15.0);
    camera_width_ = declare_parameter<int>("camera_width", 640);
    camera_height_ = declare_parameter<int>("camera_height", 480);
    camera_frame_id_ = declare_parameter<std::string>(
        "camera_frame_id", "go1_camera_color_optical_frame");
    enable_sim_gps_ = declare_parameter<bool>("enable_sim_gps", true);
    gps_rate_hz_ = declare_parameter<double>("gps_rate_hz", 10.0);
    gps_frame_id_ = declare_parameter<std::string>("gps_frame_id", "go1_gps_link");
    gps_origin_latitude_ = declare_parameter<double>("gps_origin_latitude", 42.2808);
    gps_origin_longitude_ = declare_parameter<double>("gps_origin_longitude", -83.7430);
    gps_origin_altitude_ = declare_parameter<double>("gps_origin_altitude", 250.0);
    gps_horizontal_stddev_m_ = declare_parameter<double>("gps_horizontal_stddev_m", 0.8);
    gps_vertical_stddev_m_ = declare_parameter<double>("gps_vertical_stddev_m", 1.5);

    loadModel();
    initializeJointMetadata();
    initializeSensorMetadata();

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    low_cmd_echo_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("low_cmd_echo", 10);
    if (enable_sim_camera_) {
      camera_pub_ = create_publisher<sensor_msgs::msg::Image>(
        "camera/color/image_raw", rclcpp::SensorDataQoS());
      camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        "camera/color/camera_info", rclcpp::SensorDataQoS());
    }
    if (enable_sim_gps_) {
      gps_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(
        "gps/fix", rclcpp::SensorDataQoS());
    }
    ground_truth_pub_ = create_publisher<nav_msgs::msg::Odometry>("ground_truth/odom", 10);

    startRenderThread();

    low_cmd_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        "low_cmd", 10,
        [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
          handleLowCommand(*msg);
        });

    sim_timer_ = create_wall_timer(
        periodFromHz(sim_rate_hz, 500.0),
        [this]() {
          stepSimulation();
        });

    publish_timer_ = create_wall_timer(
        periodFromHz(publish_rate_hz, 100.0),
        [this]() {
          publishState();
        });

    if (enable_sim_gps_) {
      gps_timer_ = create_wall_timer(
          periodFromHz(gps_rate_hz_, 10.0),
          [this]() {
            publishGps();
          });
    }

    RCLCPP_INFO(get_logger(), "Loaded MuJoCo model: %s", model_path_.c_str());
    RCLCPP_INFO(get_logger(), "MuJoCo viewer: %s", enable_viewer_ ? "enabled" : "disabled");
    RCLCPP_INFO(
        get_logger(), "Simulated RGB camera: %s at %.1f Hz; GPS: %s at %.1f Hz",
        enable_sim_camera_ ? "enabled" : "disabled", camera_rate_hz_,
        enable_sim_gps_ ? "enabled" : "disabled", gps_rate_hz_);
    RCLCPP_INFO(
        get_logger(),
        "Simulation command topic expects 60 floats: q[12], dq[12], kp[12], kd[12], tau[12].");
  }

  ~Go1MujocoSimNode() override
  {
    shutdownViewer();
    if (data_) {
      mj_deleteData(data_);
    }
    if (model_) {
      mj_deleteModel(model_);
    }
  }

private:
  void loadModel()
  {
    char error[1024] = {0};
    model_ = mj_loadXML(model_path_.c_str(), nullptr, error, sizeof(error));
    if (!model_) {
      throw std::runtime_error(std::string("Failed to load MuJoCo XML: ") + error);
    }

    data_ = mj_makeData(model_);
    if (!data_) {
      throw std::runtime_error("Failed to allocate MuJoCo data");
    }

    if (model_->nu < static_cast<int>(kNumJoints)) {
      throw std::runtime_error("MuJoCo model has fewer than 12 actuators");
    }
  }

  void initializeJointMetadata()
  {
    joint_names_.clear();
    joint_qpos_addr_.clear();
    joint_qvel_addr_.clear();

    for (int joint_id = 0; joint_id < model_->njnt; ++joint_id) {
      if (model_->jnt_type[joint_id] == mjJNT_FREE) {
        continue;
      }
      if (joint_names_.size() >= kNumJoints) {
        break;
      }

      const char * name = mj_id2name(model_, mjOBJ_JOINT, joint_id);
      joint_names_.push_back(name ? std::string(name) : "joint_" + std::to_string(joint_id));
      joint_qpos_addr_.push_back(model_->jnt_qposadr[joint_id]);
      joint_qvel_addr_.push_back(model_->jnt_dofadr[joint_id]);
    }

    if (joint_names_.size() != kNumJoints) {
      throw std::runtime_error("Expected 12 non-free joints in the Go1 MuJoCo model");
    }
  }

  void initializeSensorMetadata()
  {
    accel_sensor_id_ = mj_name2id(model_, mjOBJ_SENSOR, "accelerometer");
    gyro_sensor_id_ = mj_name2id(model_, mjOBJ_SENSOR, "gyro");
    if (enable_sim_camera_) {
      camera_id_ = mj_name2id(model_, mjOBJ_CAMERA, camera_name_.c_str());
      if (camera_id_ < 0) {
        throw std::runtime_error("MuJoCo camera not found: " + camera_name_);
      }
      if (camera_width_ <= 0 || camera_height_ <= 0 ||
          camera_width_ > model_->vis.global.offwidth ||
          camera_height_ > model_->vis.global.offheight) {
        throw std::runtime_error(
                "Camera dimensions must be positive and no larger than MuJoCo offscreen dimensions");
      }
    }
  }

  void startRenderThread()
  {
    if (!enable_viewer_ && !enable_sim_camera_) {
      return;
    }

    viewer_running_.store(true);
    viewer_thread_ = std::thread([this]() {
      viewerLoop();
    });
  }

  void viewerLoop()
  {
    if (!glfwInit()) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize GLFW for MuJoCo viewer");
      viewer_running_.store(false);
      return;
    }
    glfw_initialized_ = true;

    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_VISIBLE, enable_viewer_ ? GLFW_TRUE : GLFW_FALSE);
    window_ = glfwCreateWindow(
        viewer_width_, viewer_height_, "Go1 MuJoCo ROS Simulation", nullptr, nullptr);
    if (!window_) {
      RCLCPP_ERROR(get_logger(), "Failed to create GLFW window for MuJoCo viewer");
      glfwTerminate();
      glfw_initialized_ = false;
      viewer_running_.store(false);
      return;
    }

    glfwMakeContextCurrent(window_);
    glfwSwapInterval(0);

    mjv_defaultCamera(&camera_);
    mjv_defaultOption(&option_);
    mjv_defaultScene(&scene_);
    mjr_defaultContext(&context_);

    camera_.azimuth = 90.0;
    camera_.elevation = -25.0;
    camera_.distance = 4.0;
    camera_.lookat[0] = 0.0;
    camera_.lookat[1] = 0.0;
    camera_.lookat[2] = 0.5;

    mjv_makeScene(model_, &scene_, 2000);
    mjr_makeContext(model_, &context_, mjFONTSCALE_150);
    viewer_initialized_ = true;

    const auto camera_period = periodFromHz(camera_rate_hz_, 15.0);
    auto next_camera_frame = std::chrono::steady_clock::now();

    while (viewer_running_.load()) {
      if (enable_viewer_ && !renderViewerFrame()) {
        break;
      }
      const auto current_time = std::chrono::steady_clock::now();
      if (enable_sim_camera_ && current_time >= next_camera_frame) {
        renderCameraFrame();
        next_camera_frame = current_time + camera_period;
      }
      glfwPollEvents();
      std::this_thread::sleep_for(enable_viewer_ ? render_period_ : 1ms);
    }

    freeViewerResources();
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
      std::lock_guard<std::mutex> lock(command_mutex_);
      for (size_t i = 0; i < kNumJoints; ++i) {
        q_des_[i] = msg.data[i];
        dq_des_[i] = msg.data[12 + i];
        kp_[i] = msg.data[24 + i];
        kd_[i] = msg.data[36 + i];
        tau_ff_[i] = msg.data[48 + i];
      }
      have_command_ = true;
      last_command_time_ = now();
    }

    low_cmd_echo_pub_->publish(msg);
  }

  void stepSimulation()
  {
    std::lock_guard<std::mutex> sim_lock(sim_mutex_);
    applyCommand();
    mj_step(model_, data_);
  }

  void applyCommand()
  {
    std::lock_guard<std::mutex> command_lock(command_mutex_);

    if (!have_command_ || (now() - last_command_time_).seconds() > command_timeout_s_) {
      std::fill(data_->ctrl, data_->ctrl + model_->nu, 0.0);
      return;
    }

    for (size_t i = 0; i < kNumJoints; ++i) {
      const double q = data_->qpos[joint_qpos_addr_[i]];
      const double dq = data_->qvel[joint_qvel_addr_[i]];
      double torque = tau_ff_[i] + kp_[i] * (q_des_[i] - q) + kd_[i] * (dq_des_[i] - dq);

      if (model_->actuator_ctrllimited[i]) {
        const double lo = model_->actuator_ctrlrange[2 * i];
        const double hi = model_->actuator_ctrlrange[2 * i + 1];
        torque = std::clamp(torque, lo, hi);
      }

      data_->ctrl[i] = torque;
    }
  }

  void publishState()
  {
    std::lock_guard<std::mutex> sim_lock(sim_mutex_);

    if (!publish_when_paused_ && data_->time <= 0.0) {
      return;
    }

    const auto stamp = now();

    sensor_msgs::msg::JointState joint_msg;
    joint_msg.header.stamp = stamp;
    joint_msg.name = joint_names_;
    joint_msg.position.resize(kNumJoints);
    joint_msg.velocity.resize(kNumJoints);
    joint_msg.effort.resize(kNumJoints);

    for (size_t i = 0; i < kNumJoints; ++i) {
      joint_msg.position[i] = data_->qpos[joint_qpos_addr_[i]];
      joint_msg.velocity[i] = data_->qvel[joint_qvel_addr_[i]];
      joint_msg.effort[i] = data_->qfrc_actuator[joint_qvel_addr_[i]];
    }
    joint_state_pub_->publish(joint_msg);

    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = stamp;
    imu_msg.header.frame_id = "go1_imu";
    imu_msg.orientation.w = data_->qpos[3];
    imu_msg.orientation.x = data_->qpos[4];
    imu_msg.orientation.y = data_->qpos[5];
    imu_msg.orientation.z = data_->qpos[6];

    if (gyro_sensor_id_ >= 0) {
      const mjtNum * gyro = data_->sensordata + model_->sensor_adr[gyro_sensor_id_];
      imu_msg.angular_velocity.x = gyro[0];
      imu_msg.angular_velocity.y = gyro[1];
      imu_msg.angular_velocity.z = gyro[2];
    }

    if (accel_sensor_id_ >= 0) {
      const mjtNum * accel = data_->sensordata + model_->sensor_adr[accel_sensor_id_];
      imu_msg.linear_acceleration.x = accel[0];
      imu_msg.linear_acceleration.y = accel[1];
      imu_msg.linear_acceleration.z = accel[2];
    }

    imu_pub_->publish(imu_msg);
  }

  bool renderViewerFrame()
  {
    if (!viewer_initialized_ || !window_) {
      return false;
    }

    if (glfwWindowShouldClose(window_)) {
      RCLCPP_INFO(get_logger(), "MuJoCo viewer closed; shutting down ROS node.");
      rclcpp::shutdown();
      return false;
    }

    int width = 0;
    int height = 0;
    glfwGetFramebufferSize(window_, &width, &height);
    mjrRect viewport{0, 0, width, height};

    std::string overlay;
    {
      std::lock_guard<std::mutex> sim_lock(sim_mutex_);
      mjv_updateScene(model_, data_, &option_, nullptr, &camera_, mjCAT_ALL, &scene_);
      overlay = "time " + std::to_string(data_->time);
    }

    mjr_render(viewport, &scene_, &context_);
    mjr_overlay(
        mjFONT_NORMAL, mjGRID_TOPLEFT, viewport,
        overlay.c_str(), "close window to stop node", &context_);

    glfwSwapBuffers(window_);
    return true;
  }

  void renderCameraFrame()
  {
    mjvCamera sensor_camera;
    mjv_defaultCamera(&sensor_camera);
    sensor_camera.type = mjCAMERA_FIXED;
    sensor_camera.fixedcamid = camera_id_;

    mjr_setBuffer(mjFB_OFFSCREEN, &context_);
    const mjrRect viewport{0, 0, camera_width_, camera_height_};
    {
      std::lock_guard<std::mutex> sim_lock(sim_mutex_);
      mjv_updateScene(
          model_, data_, &option_, nullptr, &sensor_camera, mjCAT_ALL, &scene_);
    }
    mjr_render(viewport, &scene_, &context_);

    std::vector<unsigned char> bottom_up(
        static_cast<size_t>(camera_width_ * camera_height_ * 3));
    mjr_readPixels(bottom_up.data(), nullptr, viewport, &context_);

    sensor_msgs::msg::Image image;
    image.header.stamp = now();
    image.header.frame_id = camera_frame_id_;
    image.height = static_cast<uint32_t>(camera_height_);
    image.width = static_cast<uint32_t>(camera_width_);
    image.encoding = "rgb8";
    image.is_bigendian = false;
    image.step = static_cast<sensor_msgs::msg::Image::_step_type>(camera_width_ * 3);
    image.data.resize(bottom_up.size());
    const size_t row_bytes = static_cast<size_t>(camera_width_ * 3);
    for (int row = 0; row < camera_height_; ++row) {
      std::copy_n(
          bottom_up.data() + static_cast<size_t>(camera_height_ - 1 - row) * row_bytes,
          row_bytes, image.data.data() + static_cast<size_t>(row) * row_bytes);
    }

    sensor_msgs::msg::CameraInfo info;
    info.header = image.header;
    info.height = image.height;
    info.width = image.width;
    info.distortion_model = "plumb_bob";
    info.d.assign(5, 0.0);
    const double fovy_rad = model_->cam_fovy[camera_id_] * mjPI / 180.0;
    const double fy = 0.5 * camera_height_ / std::tan(0.5 * fovy_rad);
    const double fx = fy;
    const double cx = (camera_width_ - 1) * 0.5;
    const double cy = (camera_height_ - 1) * 0.5;
    info.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
    info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    info.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};

    camera_pub_->publish(std::move(image));
    camera_info_pub_->publish(std::move(info));
    if (enable_viewer_) {
      mjr_setBuffer(mjFB_WINDOW, &context_);
    }
  }

  void publishGps()
  {
    constexpr double kEarthRadiusM = 6378137.0;
    constexpr double kDegreesPerRadian = 180.0 / mjPI;

    std::lock_guard<std::mutex> sim_lock(sim_mutex_);
    const auto stamp = now();
    const double east = data_->qpos[0];
    const double north = data_->qpos[1];
    const double up = data_->qpos[2];

    std::normal_distribution<double> horizontal_noise(0.0, gps_horizontal_stddev_m_);
    std::normal_distribution<double> vertical_noise(0.0, gps_vertical_stddev_m_);
    const double noisy_east = east + horizontal_noise(random_engine_);
    const double noisy_north = north + horizontal_noise(random_engine_);
    const double noisy_up = up + vertical_noise(random_engine_);

    sensor_msgs::msg::NavSatFix fix;
    fix.header.stamp = stamp;
    fix.header.frame_id = gps_frame_id_;
    fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    fix.latitude = gps_origin_latitude_ +
      noisy_north / kEarthRadiusM * kDegreesPerRadian;
    const double origin_latitude_rad = gps_origin_latitude_ / kDegreesPerRadian;
    fix.longitude = gps_origin_longitude_ +
      noisy_east / (kEarthRadiusM * std::cos(origin_latitude_rad)) * kDegreesPerRadian;
    fix.altitude = gps_origin_altitude_ + noisy_up;
    fix.position_covariance = {
      gps_horizontal_stddev_m_ * gps_horizontal_stddev_m_, 0.0, 0.0,
      0.0, gps_horizontal_stddev_m_ * gps_horizontal_stddev_m_, 0.0,
      0.0, 0.0, gps_vertical_stddev_m_ * gps_vertical_stddev_m_};
    fix.position_covariance_type =
      sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    gps_pub_->publish(fix);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "map";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = east;
    odom.pose.pose.position.y = north;
    odom.pose.pose.position.z = up;
    odom.pose.pose.orientation.w = data_->qpos[3];
    odom.pose.pose.orientation.x = data_->qpos[4];
    odom.pose.pose.orientation.y = data_->qpos[5];
    odom.pose.pose.orientation.z = data_->qpos[6];
    odom.twist.twist.linear.x = data_->qvel[0];
    odom.twist.twist.linear.y = data_->qvel[1];
    odom.twist.twist.linear.z = data_->qvel[2];
    odom.twist.twist.angular.x = data_->qvel[3];
    odom.twist.twist.angular.y = data_->qvel[4];
    odom.twist.twist.angular.z = data_->qvel[5];
    ground_truth_pub_->publish(odom);
  }

  void shutdownViewer()
  {
    viewer_running_.store(false);
    if (viewer_thread_.joinable()) {
      viewer_thread_.join();
    }
  }

  void freeViewerResources()
  {
    if (viewer_initialized_) {
      mjr_freeContext(&context_);
      mjv_freeScene(&scene_);
      viewer_initialized_ = false;
    }
    if (window_) {
      glfwDestroyWindow(window_);
      window_ = nullptr;
    }
    if (glfw_initialized_) {
      glfwTerminate();
      glfw_initialized_ = false;
    }
  }

  std::string model_path_;
  mjModel * model_ = nullptr;
  mjData * data_ = nullptr;

  std::vector<std::string> joint_names_;
  std::vector<int> joint_qpos_addr_;
  std::vector<int> joint_qvel_addr_;
  int accel_sensor_id_ = -1;
  int gyro_sensor_id_ = -1;
  int camera_id_ = -1;

  bool enable_sim_camera_ = true;
  std::string camera_name_ = "front_camera";
  double camera_rate_hz_ = 15.0;
  int camera_width_ = 640;
  int camera_height_ = 480;
  std::string camera_frame_id_ = "go1_camera_color_optical_frame";

  bool enable_sim_gps_ = true;
  double gps_rate_hz_ = 10.0;
  std::string gps_frame_id_ = "go1_gps_link";
  double gps_origin_latitude_ = 42.2808;
  double gps_origin_longitude_ = -83.7430;
  double gps_origin_altitude_ = 250.0;
  double gps_horizontal_stddev_m_ = 0.8;
  double gps_vertical_stddev_m_ = 1.5;
  std::mt19937 random_engine_{std::random_device{}()};

  std::array<double, kNumJoints> q_des_{};
  std::array<double, kNumJoints> dq_des_{};
  std::array<double, kNumJoints> kp_{};
  std::array<double, kNumJoints> kd_{};
  std::array<double, kNumJoints> tau_ff_{};

  std::mutex sim_mutex_;
  std::mutex command_mutex_;
  bool have_command_ = false;
  double command_timeout_s_ = 0.2;
  bool publish_when_paused_ = true;
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};

  bool enable_viewer_ = false;
  int viewer_width_ = 1200;
  int viewer_height_ = 900;
  std::chrono::nanoseconds render_period_{std::chrono::milliseconds(33)};
  std::atomic<bool> viewer_running_{false};
  std::thread viewer_thread_;
  bool glfw_initialized_ = false;
  bool viewer_initialized_ = false;
  GLFWwindow * window_ = nullptr;
  mjvCamera camera_{};
  mjvOption option_{};
  mjvScene scene_{};
  mjrContext context_{};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr low_cmd_echo_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ground_truth_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr low_cmd_sub_;
  rclcpp::TimerBase::SharedPtr sim_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr gps_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go1MujocoSimNode>());
  rclcpp::shutdown();
  return 0;
}