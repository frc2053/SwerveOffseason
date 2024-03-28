// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include "constants/VisionConstants.h"
#include "str/Camera.h"

namespace str {
class Vision {
 public:
  Vision() = default;
  void SimulationPeriodic(frc::Pose2d simRobotPose);
  std::vector<std::optional<photon::EstimatedRobotPose>> GetCameraEstimatedPoses();
  std::vector<std::optional<Eigen::Matrix<double, 3, 1>>> GetPoseStdDevs(const std::vector<std::optional<photon::EstimatedRobotPose>>& poses);
 private:
  std::array<Camera, 4> cameras{
    Camera{consts::vision::FL_CAM_NAME, consts::vision::FL_ROBOT_TO_CAM, consts::vision::SINGLE_TAG_STD_DEV, consts::vision::MULTI_TAG_STD_DEV},
    Camera{consts::vision::FR_CAM_NAME, consts::vision::FR_ROBOT_TO_CAM, consts::vision::SINGLE_TAG_STD_DEV, consts::vision::MULTI_TAG_STD_DEV},
    Camera{consts::vision::BL_CAM_NAME, consts::vision::BL_ROBOT_TO_CAM, consts::vision::SINGLE_TAG_STD_DEV, consts::vision::MULTI_TAG_STD_DEV},
    Camera{consts::vision::BR_CAM_NAME, consts::vision::BR_ROBOT_TO_CAM, consts::vision::SINGLE_TAG_STD_DEV, consts::vision::MULTI_TAG_STD_DEV}
  };
};
}