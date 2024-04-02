#pragma once

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <units/time.h>
#include <units/frequency.h>

namespace consts {

inline constexpr units::second_t LOOP_PERIOD = 1 / 250_Hz;

namespace yearSpecific {
    inline const frc::AprilTagFieldLayout aprilTagLayout = frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo);
}
}