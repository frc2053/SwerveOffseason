// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/system/plant/DCMotor.h>
#include <units/base.h>
#include <units/velocity.h>

#include "str/Gains.h"

namespace consts {
namespace shooter {
namespace can_ids {
inline constexpr int TOP_SHOOTER = 15;
inline constexpr int BOTTOM_SHOOTER = 16;
} // namespace can_ids

namespace current_limits {
inline constexpr units::ampere_t SUPPLY_CURRENT_LIMIT = 60_A;
inline constexpr units::ampere_t STATOR_CURRENT_LIMIT = 180_A;
} // namespace current_limits

namespace physical {

inline constexpr bool TOP_INVERT = false;
inline constexpr bool BOTTOM_INVERT = true;

inline constexpr units::scalar_t SHOOTER_RATIO = (1.0 / 1.0);

inline constexpr frc::DCMotor SHOOTER_MOTOR = frc::DCMotor::Falcon500(1);

inline constexpr units::meter_t WHEEL_RADIUS = 2_in;

//From onshape doc
static constexpr units::kilogram_square_meter_t FLYWHEEL_MOI = 62.564007 * 1_in * 1_in * 1_lb;
} // namespace physical

namespace gains {
inline constexpr units::radians_per_second_t VEL_TOLERANCE = 10_rpm;
inline constexpr str::gains::radial::turn_volt_ka_unit_t SHOOTER_KA{0};
inline constexpr str::gains::radial::turn_amp_kv_unit_t SHOOTER_KV{0};
inline constexpr units::ampere_t SHOOTER_KS{0};
inline constexpr units::volt_t SHOOTER_KS_V{0};
inline constexpr str::gains::radial::turn_amp_kp_unit_t SHOOTER_KP{0};
inline constexpr str::gains::radial::turn_amp_ki_unit_t SHOOTER_KI{0};
inline constexpr str::gains::radial::turn_amp_kd_unit_t SHOOTER_KD{0};
} // namespace gains
} // namespace shooter
} // namespace consts
