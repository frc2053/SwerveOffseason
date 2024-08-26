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
inline constexpr frc::Translation2d inFrontOfAmpLocation{1.838_m, 7.172_m};
inline constexpr frc::Translation2d ampLocation{1.838_m, 7.782_m};
inline constexpr units::meter_t closeToAmpDistance{5_ft};
} // namespace yearSpecific
} // namespace consts
