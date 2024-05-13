#pragma once

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <units/time.h>
#include <units/frequency.h>

namespace consts {

inline constexpr units::second_t LOOP_PERIOD = 1 / 50_Hz;
inline constexpr units::second_t SWERVE_ODOM_LOOP_PERIOD = 1 / 250_Hz;

namespace yearSpecific {
    inline const frc::AprilTagFieldLayout aprilTagLayout = frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo);
    inline constexpr frc::Translation2d inFrontOfAmpLocation{1.838_m, 7.172_m};
    inline constexpr frc::Translation2d ampLocation{1.838_m, 7.782_m};
    inline constexpr units::meter_t closeToAmpDistance{5_ft};
}
}