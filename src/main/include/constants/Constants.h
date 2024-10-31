// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <units/frequency.h>
#include <units/time.h>

namespace consts {

inline constexpr units::second_t LOOP_PERIOD = 1 / 50_Hz;
inline constexpr units::second_t SWERVE_ODOM_LOOP_PERIOD = 1 / 250_Hz;

namespace yearSpecific {
inline const frc::AprilTagFieldLayout aprilTagLayout =
    frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2024Crescendo);
inline constexpr frc::Translation2d inFrontOfAmpLocation{1.838_m, 7.239_m};
inline constexpr frc::Translation2d ampLocation{1.838_m, 7.882_m};
inline constexpr units::meter_t closeToAmpDistance{5_ft};
inline constexpr frc::Translation2d passPosition{8.226_m, 0.615_m};
inline constexpr frc::Translation2d speakerLocationCenter{0_ft, 5.57_m};
inline constexpr frc::Translation2d speakerLocationAmpSide{0_ft, 5.9_m};
inline constexpr frc::Translation2d speakerLocationSourceSide{0_ft, 5.31_m};
}  // namespace yearSpecific
}  // namespace consts
