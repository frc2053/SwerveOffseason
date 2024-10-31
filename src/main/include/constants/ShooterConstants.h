// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/system/plant/DCMotor.h>
#include <units/base.h>
#include <units/velocity.h>
#include <wpi/interpolating_map.h>

#include "str/Gains.h"

namespace consts {
namespace shooter {
namespace can_ids {
inline constexpr int TOP_SHOOTER = 15;
inline constexpr int BOTTOM_SHOOTER = 16;
}  // namespace can_ids

namespace current_limits {
inline constexpr units::ampere_t SUPPLY_CURRENT_LIMIT = 60_A;
inline constexpr units::ampere_t STATOR_CURRENT_LIMIT = 180_A;
}  // namespace current_limits

namespace physical {

inline constexpr bool TOP_INVERT = true;
inline constexpr bool BOTTOM_INVERT = true;

inline constexpr units::scalar_t SHOOTER_RATIO = (1.0 / 1.0);

inline constexpr frc::DCMotor SHOOTER_MOTOR = frc::DCMotor::Falcon500(1);

inline constexpr units::meter_t WHEEL_RADIUS = 2_in;

// From onshape doc
inline constexpr units::kilogram_square_meter_t FLYWHEEL_MOI =
    5.01 * 1_in * 1_in * 1_lb;
}  // namespace physical

namespace gains {
inline constexpr units::turns_per_second_t VEL_TOLERANCE = 150_rpm;
inline constexpr str::gains::radial::turn_volt_ka_unit_t SHOOTER_KA{0.026523};
inline constexpr str::gains::radial::turn_volt_kv_unit_t SHOOTER_KV{0.11807};
inline constexpr units::volt_t SHOOTER_KS{0.069075};
inline constexpr str::gains::radial::turn_volt_kp_unit_t SHOOTER_KP{0.16309};
inline constexpr str::gains::radial::turn_volt_ki_unit_t SHOOTER_KI{0};
inline constexpr str::gains::radial::turn_volt_kd_unit_t SHOOTER_KD{0};
}  // namespace gains

struct ShooterSpeeds {
  units::turns_per_second_t topSpeed;
  units::turns_per_second_t bottomSpeed;
};

inline constexpr ShooterSpeeds SPIT_SPEEDS{850_rpm, 850_rpm};
inline constexpr ShooterSpeeds AMP_SPEEDS{750_rpm, 2250_rpm};
inline constexpr ShooterSpeeds SUBWOOFER_SPEEDS{2053_rpm, 3800_rpm};
inline constexpr ShooterSpeeds PASS_SPEEDS{2053_rpm, 3500_rpm};

struct MeterHash {
  size_t operator()(const units::meter_t& m) const {
    return std::hash<double>{}(m.value());
  }
};

inline static wpi::interpolating_map<units::meter_t, units::turns_per_second_t>
    TOP_SHOOTER_LUT{};
inline static wpi::interpolating_map<units::meter_t, units::turns_per_second_t>
    BOTTOM_SHOOTER_LUT{};

enum class PRESET_SPEEDS { OFF, AMP, SPEAKER_DIST, SUBWOOFER, PASS, TUNING, SPIT };
}  // namespace shooter
}  // namespace consts
