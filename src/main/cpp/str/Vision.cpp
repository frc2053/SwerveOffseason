#include "str/Vision.h"

using namespace str;

Vision::Vision() {

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
