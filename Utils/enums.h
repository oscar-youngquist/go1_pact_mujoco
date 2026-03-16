#pragma once

#include <iostream>

namespace perception {

enum class StateEstMode {
  CheaterMode = 0,
  VecNavOrie = 1,
  KF_BodyPosVel = 2,
  IKF_BodyPosture = 3,
  MSCKF_JointImu = 4,
  MSCKF_RgbImu = 5,
  MSCKF_RgbJointImu = 6,
  NUM_OBS
};

// Setup required sensors for each state estimation mode
const std::map<StateEstMode, std::vector<std::string>> sensor_requirements = {
  {StateEstMode::CheaterMode, {"imu", "joint", "pose"}},
  {StateEstMode::VecNavOrie, {"imu", "pose"}},
  {StateEstMode::KF_BodyPosVel, {"imu", "joint"}},
  {StateEstMode::IKF_BodyPosture, {"imu", "joint"}},
  {StateEstMode::MSCKF_JointImu, {"imu", "joint"}},
  {StateEstMode::MSCKF_RgbImu, {"imu", "rgbd"}},
  {StateEstMode::MSCKF_RgbJointImu, {"imu", "rgbd", "joint"}}
};

inline StateEstMode GetStateEstModeFromInteger(int modeInt) {
  if (modeInt >= 0 && modeInt < static_cast<int>(StateEstMode::NUM_OBS)) {
    return static_cast<StateEstMode>(modeInt);
  }

  // invalid input
  std::cerr << "Error: Invalid mode " << modeInt << ". Allowed range: [0, " 
            << static_cast<int>(StateEstMode::NUM_OBS) - 1 << "]" << std::endl;
  std::exit(EXIT_FAILURE);
}


inline std::string StateEstModeToString(StateEstMode mode) {
  switch (mode) {
    case StateEstMode::CheaterMode: return "CheaterMode";
    case StateEstMode::VecNavOrie: return "VecNavOrie";
    case StateEstMode::KF_BodyPosVel: return "KF_BodyPosVel";
    case StateEstMode::IKF_BodyPosture: return "IKF_BodyPosture";
    case StateEstMode::MSCKF_JointImu: return "MSCKF_JointImu";
    case StateEstMode::MSCKF_RgbImu: return "MSCKF_RgbImu";
    case StateEstMode::MSCKF_RgbJointImu: return "MSCKF_RgbJointImu";
    default: return "UNKNOWN";
  }
}

enum class DataSource {
  SIMULATION, // from simulation
  OFFLINE,    // from local computer
  ONLINE      // from real robot
};

inline const char* DataSourceToString(DataSource source) {
  switch (source) {
    case DataSource::SIMULATION: return "SIMULATION";
    case DataSource::OFFLINE: return "OFFLINE";
    case DataSource::ONLINE: return "ONLINE";
    default: return "UNKNOWN";
  }
}


/// IMU intrinsic models
enum ImuModel {
  KALIBR = 0,
  RPNG = 1
};

/// Robot type
enum class RobotType {
  MiniCheetah = 0,
  Go1 = 1,
  Go2 = 2,
  Staccato = 3,
  Prestoe = 4,
  Unknown = 5
};

enum class DisturbanceType {
  Left = 0,
  Right = 1,
};

enum class StateCentricType {
  Body = 0,
  World = 1,
};
}