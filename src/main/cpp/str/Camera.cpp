// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "str/Camera.h"

#include <frc/RobotBase.h>
#include <optional>
#include <vector>
#include "constants/VisionConstants.h"

#include "constants/Constants.h"
#include "frc/geometry/Pose3d.h"
#include "frc/geometry/Rotation3d.h"
#include "frc/geometry/Translation3d.h"
#include "photon/estimation/TargetModel.h"
#include "photon/simulation/VisionTargetSim.h"
#include "units/angle.h"

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

      if(cameraName == consts::vision::NOTE_CAM_NAME) {
        cameraProps->SetCalibration(640, 480, frc::Rotation2d{75_deg});
      }
      else {
        cameraProps->SetCalibration(1600, 1200, frc::Rotation2d{75_deg});
      }
      cameraProps->SetCalibError(.35, .10);
      cameraProps->SetFPS(45_Hz);
      cameraProps->SetAvgLatency(20_ms);
      cameraProps->SetLatencyStdDev(15_ms);

      cameraSim = std::make_shared<photon::PhotonCameraSim>(camera.get(),
                                                            *cameraProps.get());

      visionSim->AddCamera(cameraSim.get(), robotToCamera);
      cameraSim->EnableDrawWireframe(true);

      photon::TargetModel notePlaceHolder{
          CreateTorusVertices(14_in, 1_in, 5, 5)};
      visionSim->AddVisionTargets({photon::VisionTargetSim{
          frc::Pose3d{14_ft, 14_ft, 0_ft, frc::Rotation3d{}},
          notePlaceHolder}});
    }
  }
}

std::optional<units::meter_t> Camera::GetDistanceToNote() {
  double max_area = 0;
  double max_width = 0;
  double best_yaw = 0;
  double pixelToRad = 20;
  bool didReadData = false;
  for (const auto &result : camera->GetAllUnreadResults()) {
    didReadData = true;
    if (result.HasTargets()) {
      for (const auto &target : result.GetTargets()) {
        if (target.GetFiducialId() == -1) {
          double width = 0;
          double yaw = 0;
          std::vector<photon::TargetCorner> corners =
              target.GetDetectedCorners();
          double minX = corners[0].x;
          double maxX = corners[0].x;

          for (const auto &corner : corners) {
            minX = std::min(minX, corner.x);
            maxX = std::max(maxX, corner.x);
          }

          width = maxX - minX;
          yaw = target.GetYaw();

          if(target.GetArea() > max_area) {
            max_area = target.GetArea();
            max_width = width;
            best_yaw = yaw;
          }
        }
      }
    }
  }
  if(!didReadData) {
    fmt::print("data not found!\n");
  }
  if (max_width != 0) {
    angleToNote = std::make_optional(units::degree_t{best_yaw});
    fmt::print("max width: {}\n", max_width);
    units::meter_t dist = 14_in / units::math::tan(units::radian_t{max_width / (cameraProps->GetResWidth() / cameraProps->GetHorizFOV().Radians())});
    return dist;
  } else {
    fmt::print("not found!\n");
    return std::nullopt;
  }
}

std::optional<units::radian_t> Camera::GetAngleToNote() {
  return angleToNote;
}

photon::PhotonPipelineResult Camera::GetLatestResult() { return latestResult; }

std::optional<photon::EstimatedRobotPose> Camera::GetEstimatedGlobalPose() {
  if (!simulate) {
    return std::nullopt;
  }

  std::optional<photon::EstimatedRobotPose> visionEst;

  for (const auto &result : camera->GetAllUnreadResults()) {
    visionEst = photonEstimator->Update(result);

    if (visionEst.has_value()) {
      posePub.Set(visionEst.value().estimatedPose.ToPose2d());
    } else {
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

std::vector<frc::Translation3d>
Camera::CreateTorusVertices(units::meter_t majorRadius,
                            units::meter_t minorRadius, int numMajorDivisions,
                            int numMinorDivisons) {
  std::vector<frc::Translation3d> vertices{};

  units::radian_t majorAngleIncrement =
      units::radian_t{2 * std::numbers::pi} / numMajorDivisions;
  units::radian_t minorAngleIncrement =
      units::radian_t{2 * std::numbers::pi} / numMinorDivisons;

  for (int i = 0; i < numMajorDivisions; i++) {
    units::radian_t majorAngle = i * majorAngleIncrement;
    for (int j = 0; j < numMinorDivisons; j++) {
      units::radian_t minorAngle = j * minorAngleIncrement;
      units::meter_t x =
          (majorRadius + minorRadius * units::math::cos(minorAngle)) *
          units::math::cos(majorAngle);
      units::meter_t y =
          (majorRadius + minorRadius * units::math::cos(minorAngle)) *
          units::math::sin(majorAngle);
      units::meter_t z = minorRadius * units::math::sin(minorAngle);
      vertices.push_back(frc::Translation3d{x, y, z});
    }
  }

  return vertices;
}
