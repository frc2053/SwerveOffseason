#pragma once

#include <string>
#include <frc/geometry/Transform3d.h>
#include <frc/apriltag/AprilTagFields.h>
#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/estimation/VisionEstimation.h>
#include <photon/simulation/VisionSystemSim.h>
#include <photon/simulation/VisionTargetSim.h>
#include <photon/targeting/PhotonPipelineResult.h>
#include <networktables/StructTopic.h>

namespace str {
class Camera {
public:
  Camera(std::string cameraName, frc::Transform3d robotToCamera, Eigen::Matrix<double, 3, 1> singleTagStdDev, Eigen::Matrix<double, 3, 1> multiTagDevs, bool simulate);
  void SimPeriodic(frc::Pose2d robotSimPose);
  photon::PhotonPipelineResult GetLatestResult();
  std::optional<photon::EstimatedRobotPose> GetEstimatedGlobalPose();
  Eigen::Matrix<double, 3, 1> GetEstimationStdDevs(frc::Pose2d estimatedPose);
private:
  bool simulate;
  std::unique_ptr<photon::PhotonPoseEstimator> photonEstimator;
  std::shared_ptr<photon::PhotonCamera> camera;
  std::unique_ptr<photon::VisionSystemSim> visionSim;
  std::unique_ptr<photon::SimCameraProperties> cameraProps;
  std::shared_ptr<photon::PhotonCameraSim> cameraSim;
  units::second_t lastEstTimestamp{0_s};

  Eigen::Matrix<double, 3, 1> singleTagDevs;
  Eigen::Matrix<double, 3, 1> multiTagDevs;

  std::shared_ptr<nt::NetworkTable> nt{nt::NetworkTableInstance::GetDefault().GetTable("Vision")};
  nt::StructPublisher<frc::Pose2d> posePub;
  nt::DoublePublisher stdDevXPosePub;
  nt::DoublePublisher stdDevYPosePub;
  nt::DoublePublisher stdDevRotPosePub;
};
}