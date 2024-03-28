#include "str/Vision.h"

using namespace str;

std::vector<std::optional<Eigen::Matrix<double, 3, 1>>> Vision::GetPoseStdDevs(const std::vector<std::optional<photon::EstimatedRobotPose>>& poses) {
    std::vector<std::optional<Eigen::Matrix<double, 3, 1>>> allStdDevs;
    int i = 0;
    for(auto& cam : cameras) {
        if(poses[i].has_value()) {
            allStdDevs.push_back(cam.GetEstimationStdDevs(poses[i].value().estimatedPose.ToPose2d()));
        }
        else {
            allStdDevs.push_back(std::nullopt);
        }
        i++;
    }
    return allStdDevs;
}

std::vector<std::optional<photon::EstimatedRobotPose>> Vision::GetCameraEstimatedPoses() {
    std::vector<std::optional<photon::EstimatedRobotPose>> allPoses;
    for(auto& cam : cameras) {
        allPoses.push_back(cam.GetEstimatedGlobalPose());
    }
    return allPoses;
}

void Vision::SimulationPeriodic(frc::Pose2d simRobotPose) {
    for(auto& cam : cameras) {
        cam.SimPeriodic(simRobotPose);
    }
}
