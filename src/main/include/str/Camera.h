// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/apriltag/AprilTagFields.h>
#include <frc/geometry/Transform3d.h>
#include <networktables/StructTopic.h>
#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/estimation/VisionEstimation.h>
#include <photon/simulation/VisionSystemSim.h>
#include <photon/simulation/VisionTargetSim.h>
#include <photon/targeting/PhotonPipelineResult.h>

#include <memory>
#include <string>
#include <vector>

#include "frc/geometry/Translation3d.h"
#include "units/length.h"

namespace str {
class Camera {
 public:
  Camera(std::string cameraName, frc::Transform3d robotToCamera,
         Eigen::Matrix<double, 3, 1> singleTagStdDev,
         Eigen::Matrix<double, 3, 1> multiTagDevs, bool simulate);
  void SimPeriodic(frc::Pose2d robotSimPose);
  photon::PhotonPipelineResult GetLatestResult();
  std::optional<photon::EstimatedRobotPose> GetEstimatedGlobalPose();
  Eigen::Matrix<double, 3, 1> GetEstimationStdDevs(frc::Pose2d estimatedPose);
  std::optional<units::meter_t> GetDistanceToNote();
  std::optional<units::radian_t> GetAngleToNote();

 private:
  bool simulate;
  std::unique_ptr<photon::PhotonPoseEstimator> photonEstimator;
  std::unique_ptr<photon::PhotonCamera> camera;
  std::unique_ptr<photon::VisionSystemSim> visionSim;
  std::unique_ptr<photon::SimCameraProperties> cameraProps;
  std::shared_ptr<photon::PhotonCameraSim> cameraSim;

  photon::PhotonPipelineResult latestResult;

  Eigen::Matrix<double, 3, 1> singleTagDevs;
  Eigen::Matrix<double, 3, 1> multiTagDevs;

  std::shared_ptr<nt::NetworkTable> nt{
      nt::NetworkTableInstance::GetDefault().GetTable("Vision")};
  nt::StructPublisher<frc::Pose2d> posePub;
  nt::DoublePublisher stdDevXPosePub;
  nt::DoublePublisher stdDevYPosePub;
  nt::DoublePublisher stdDevRotPosePub;

  std::optional<units::radian_t> angleToNote;

  units::radian_t cacheYaw = 0_rad;
  units::meter_t cacheDist = 0_m;

  std::vector<frc::Translation3d> CreateTorusVertices(
      units::meter_t majorRadius, units::meter_t minorRadius,
      int numMajorDivisions, int numMinorDivisons);
};
}  // namespace str
