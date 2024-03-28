#include "str/Camera.h"
#include <frc/RobotBase.h>
#include "constants/Constants.h"

using namespace str;

Camera::Camera(std::string cameraName, frc::Transform3d robotToCamera, Eigen::Matrix<double, 3, 1> singleTagStdDev, Eigen::Matrix<double, 3, 1> multiTagDevs) : 
    poseTopic(nt->GetStructTopic<frc::Pose2d>(cameraName + "PoseEstimation")),
    posePub(poseTopic.Publish()),
    singleTagDevs(singleTagDevs),
    multiTagDevs(multiTagDevs)
{
    photonEstimator = std::make_unique<photon::PhotonPoseEstimator>(
        consts::yearSpecific::aprilTagLayout, 
        photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
        std::move(photon::PhotonCamera(cameraName)),
        robotToCamera
    );
    camera = photonEstimator->GetCamera();
    camera->SetVersionCheckEnabled(false);
    photonEstimator->SetMultiTagFallbackStrategy(photon::PoseStrategy::LOWEST_AMBIGUITY);

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

photon::PhotonPipelineResult Camera::GetLatestResult() {
    return camera->GetLatestResult();
}

std::optional<photon::EstimatedRobotPose> Camera::GetEstimatedGlobalPose() {
    auto visionEst = photonEstimator->Update();
    units::second_t latestTimestamp = camera->GetLatestResult().GetTimestamp();


    //FIXME: waiting on wpilib for fix
    //account for issue with NT sync not working correctly.
    units::second_t now = frc::Timer::GetFPGATimestamp();
    if(latestTimestamp > now) {
        latestTimestamp = now;
    }

    bool newResult = units::math::abs(latestTimestamp - lastEstTimestamp) > 1e-5_s;
    if (visionEst.has_value()) {
        posePub.Set(visionEst.value().estimatedPose.ToPose2d());
    } else {
        if (newResult) {
            posePub.Set({});
        }
    }
    if (newResult) {
        lastEstTimestamp = latestTimestamp;
    }
    return visionEst;
}

Eigen::Matrix<double, 3, 1> Camera::GetEstimationStdDevs(frc::Pose2d estimatedPose) {
    Eigen::Matrix<double, 3, 1> estStdDevs = singleTagDevs;
    auto targetsfl = GetLatestResult().GetTargets();
    int numTags = 0;
    units::meter_t avgDist = 0_m;
    for (const auto& tgt : targetsfl) {
        auto tagPose = photonEstimator->GetFieldLayout().GetTagPose(tgt.GetFiducialId());
        if (tagPose.has_value()) {
        numTags++;
        avgDist += tagPose.value().ToPose2d().Translation().Distance(estimatedPose.Translation());
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
        estStdDevs = (Eigen::MatrixXd(3, 1) << std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()).finished();
    } else {
        estStdDevs = estStdDevs * (1 + (avgDist.value() * avgDist.value() / 30));
    }
    return estStdDevs;
}

void Camera::SimPeriodic(frc::Pose2d robotSimPose) {
    visionSim->Update(robotSimPose);
}