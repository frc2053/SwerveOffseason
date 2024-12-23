// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "str/Vision.h"

#include <frc/geometry/Pose2d.h>

#include <vector>

using namespace str;

std::vector<std::optional<Eigen::Matrix<double, 3, 1>>> Vision::GetPoseStdDevs(
    const std::vector<std::optional<photon::EstimatedRobotPose>>& poses) {
  std::vector<std::optional<Eigen::Matrix<double, 3, 1>>> allStdDevs;
  int i = 0;
  for (auto& cam : cameras) {
    if (poses[i].has_value()) {
      auto stddev = cam.GetEstimationStdDevs(poses[i].value().estimatedPose.ToPose2d());
      allStdDevs.push_back(stddev);
    } else {
      allStdDevs.push_back(std::nullopt);
    }
    i++;
  }
  return allStdDevs;
}

std::vector<std::optional<photon::EstimatedRobotPose>>
Vision::GetCameraEstimatedPoses(frc::Pose3d robotPose) {
  std::vector<std::optional<photon::EstimatedRobotPose>> allPoses;
  for (auto& cam : cameras) {
    allPoses.push_back(cam.GetEstimatedGlobalPose(robotPose));
  }
  return allPoses;
}

void Vision::SimulationPeriodic(frc::Pose2d simRobotPose) {
  for (auto& cam : cameras) {
    cam.SimPeriodic(simRobotPose);
    //noteCamera.SimPeriodic(simRobotPose);
  }
}

std::optional<units::meter_t> Vision::GetDistanceToNote() {
  return {};//noteCamera.GetDistanceToNote();
}

std::optional<units::radian_t> Vision::GetAngleToNote() {
  return {};//noteCamera.GetAngleToNote();
}
