// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include "str/Gains.h"

namespace consts {
namespace pivot {
namespace can_ids {
inline constexpr int LEFT_PIVOT = 15;
inline constexpr int RIGHT_PIVOT = 16;
inline constexpr int PIVOT_ENCODER = 17;
} // namespace can_ids

namespace current_limits {
inline constexpr units::ampere_t STEER_TORQUE_CURRENT_LIMIT = 40_A;
inline constexpr units::ampere_t SUPPLY_CURRENT_LIMIT = 60_A;
} // namespace current_limits

namespace physical {
inline constexpr units::scalar_t PIVOT_GEARING = 100.0 / 1.0;
inline constexpr units::turn_t ENCODER_OFFSET = 0.1_tr;
} // namespace physical

namespace gains {
inline constexpr units::turns_per_second_t motionMagicCruiseVel = 90_deg_per_s;
inline constexpr str::gains::radial::turn_volt_ka_unit_t motionMagicExpoKa{0};
inline constexpr str::gains::radial::turn_volt_kv_unit_t motionMagicExpoKv{0};
inline constexpr str::gains::radial::turn_amp_ka_unit_t kA{0};
inline constexpr str::gains::radial::turn_amp_kv_unit_t kV{0};
inline constexpr units::ampere_t kS{0};
inline constexpr units::ampere_t kG{0};
inline constexpr str::gains::radial::turn_amp_kp_unit_t kP{0};
inline constexpr str::gains::radial::turn_amp_ki_unit_t kI{0};
inline constexpr str::gains::radial::turn_amp_kd_unit_t kD{0};
} // namespace gains
} // namespace pivot
} // namespace consts
