#include <algorithm>
#include <atomic>
#include <array>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
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

    loadModel();
    initializeJointMetadata();
    initializeSensorMetadata();
    startViewerThread();

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    low_cmd_echo_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("low_cmd_echo", 10);

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



    RCLCPP_INFO(get_logger(), "Loaded MuJoCo model: %s", model_path_.c_str());
    RCLCPP_INFO(get_logger(), "MuJoCo viewer: %s", enable_viewer_ ? "enabled" : "disabled");
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
  }

  void startViewerThread()
  {
    if (!enable_viewer_) {
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

    while (viewer_running_.load()) {
      if (!renderViewerFrame()) {
        break;
      }
      std::this_thread::sleep_for(render_period_);
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
    glfwPollEvents();
    return true;
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
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr low_cmd_sub_;
  rclcpp::TimerBase::SharedPtr sim_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go1MujocoSimNode>());
  rclcpp::shutdown();
  return 0;
}