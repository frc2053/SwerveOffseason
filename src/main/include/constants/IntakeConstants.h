// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/system/plant/DCMotor.h>
#include <units/base.h>
#include <units/dimensionless.h>
#include <units/moment_of_inertia.h>
#include <units/velocity.h>
#include <wpi/interpolating_map.h>

#include "str/Gains.h"

namespace consts {
namespace intake {
namespace can_ids {
inline constexpr int INTAKE = 17;
} // namespace can_ids

namespace current_limits {
inline constexpr units::ampere_t SUPPLY_CURRENT_LIMIT = 60_A;
inline constexpr units::ampere_t STATOR_CURRENT_LIMIT = 180_A;
} // namespace current_limits

namespace physical {

inline constexpr bool INVERT_MOTOR = false;

inline constexpr units::scalar_t INTAKE_RATIO = (24.0 / 16.0);

inline constexpr frc::DCMotor INTAKE_MOTOR = frc::DCMotor::Falcon500(1);

inline constexpr units::meter_t WHEEL_RADIUS = 1_in;

// From onshape doc
inline constexpr units::kilogram_square_meter_t INTAKE_MOI =
    5.350445 * 1_in * 1_in * 1_lb;
} // namespace physical

namespace gains {
inline constexpr units::ampere_t NOTE_SPIKE_THRESHOLD = 100_A;
inline constexpr units::volt_t NOTE_INTAKE_VOLTAGE = 10_V;
inline constexpr units::volt_t NOTE_EJECT_VOLTAGE = -10_V;
} // namespace gains
} // namespace intake
} // namespace consts
