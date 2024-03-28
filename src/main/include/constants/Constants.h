#pragma once

#include <frc/apriltag/AprilTagFieldLayout.h>

namespace consts {
namespace yearSpecific {
    inline const frc::AprilTagFieldLayout aprilTagLayout = frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo);
}
}