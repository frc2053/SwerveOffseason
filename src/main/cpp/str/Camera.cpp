// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "str/Camera.h"

#include <frc/RobotBase.h>

#include "constants/Constants.h"

using namespace str;

Camera::Camera(std::string cameraName, frc::Transform3d robotToCamera,
               Eigen::Matrix<double, 3, 1> singleTagStdDev,
               Eigen::Matrix<double, 3, 1> multiTagDevs, bool simulate)
    : simulate(simulate),
      posePub(nt->GetStructTopic<frc::Pose2d>(cameraName + "PoseEstimation")
                  .Publish()),
      singleTagDevs(singleTagDevs), multiTagDevs(multiTagDevs),
      stdDevXPosePub(nt->GetDoubleTopic(cameraName + "StdDevsX").Publish()),
      stdDevYPosePub(nt->GetDoubleTopic(cameraName + "StdDevsY").Publish()),
      stdDevRotPosePub(
          nt->GetDoubleTopic(cameraName + "StdDevsRot").Publish()) {
  photonEstimator = std::make_unique<photon::PhotonPoseEstimator>(
      consts::yearSpecific::aprilTagLayout,
      photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
  camera = std::make_unique<photon::PhotonCamera>(cameraName);
  camera->SetVersionCheckEnabled(false);
  photonEstimator->SetMultiTagFallbackStrategy(
      photon::PoseStrategy::LOWEST_AMBIGUITY);

  if (simulate) {
    if (frc::RobotBase::IsSimulation()) {
      visionSim = std::make_unique<photon::VisionSystemSim>(cameraName);
      visionSim->AddAprilTags(consts::yearSpecific::aprilTagLayout);
      cameraProps = std::make_unique<photon::SimCameraProperties>();

      cameraProps->SetCalibration(1600, 1200, frc::Rotation2d{90_deg});
      cameraProps->SetCalibError(.35, .10);
      cameraProps->SetFPS(45_Hz);
      cameraProps->SetAvgLatency(20_ms);
      cameraProps->SetLatencyStdDev(15_ms);

      cameraSim = std::make_shared<photon::PhotonCameraSim>(camera.get(),
                                                            *cameraProps.get());

      visionSim->AddCamera(cameraSim.get(), robotToCamera);
      cameraSim->EnableDrawWireframe(true);
    }
  }
}

photon::PhotonPipelineResult Camera::GetLatestResult() {
  return latestResult;
}

std::optional<photon::EstimatedRobotPose> Camera::GetEstimatedGlobalPose() {
  if (!simulate) {
    return std::nullopt;
  }

  std::optional<photon::EstimatedRobotPose> visionEst;

  for(const auto& result : camera->GetAllUnreadResults()) {
    visionEst = photonEstimator->Update(result);

    if(visionEst.has_value()) {
      posePub.Set(visionEst.value().estimatedPose.ToPose2d());
    }
    else {
      posePub.Set({});
    }

    latestResult = result;
  }

  return visionEst;
}

Eigen::Matrix<double, 3, 1>
Camera::GetEstimationStdDevs(frc::Pose2d estimatedPose) {
  Eigen::Matrix<double, 3, 1> estStdDevs = singleTagDevs;
  auto targets = GetLatestResult().GetTargets();
  int numTags = 0;
  units::meter_t avgDist = 0_m;
  for (const auto &tgt : targets) {
    auto tagPose =
        photonEstimator->GetFieldLayout().GetTagPose(tgt.GetFiducialId());
    if (tagPose.has_value()) {
      numTags++;
      avgDist += tagPose.value().ToPose2d().Translation().Distance(
          estimatedPose.Translation());
    }
  }
  if (numTags == 0) {
    return estStdDevs;
  }
  avgDist /= numTags;
  if (numTags > 1) {
    estStdDevs = multiTagDevs;
  }
  if (numTags == 1 && avgDist > 4_m) {
    estStdDevs =
        (Eigen::MatrixXd(3, 1) << std::numeric_limits<double>::max(),
         std::numeric_limits<double>::max(), std::numeric_limits<double>::max())
            .finished();
  } else {
    estStdDevs = estStdDevs * (1 + (avgDist.value() * avgDist.value() / 30));
  }

  if (!simulate) {
    stdDevXPosePub.Set(estStdDevs(0));
    stdDevYPosePub.Set(estStdDevs(1));
    stdDevRotPosePub.Set(estStdDevs(2));
  }

  return estStdDevs;
}

void Camera::SimPeriodic(frc::Pose2d robotSimPose) {
  if (simulate) {
    visionSim->Update(robotSimPose);
  }
}
