// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/time.h>

namespace str {
namespace gains {
namespace linear {} // namespace linear

namespace radial {
using radial_velocity =
    units::compound_unit<units::turns, units::inverse<units::seconds>>;
using radial_accel =
    units::compound_unit<radial_velocity, units::inverse<units::seconds>>;

using turn_volt_ka_unit =
    units::compound_unit<units::volts, units::inverse<radial_accel>>;
using turn_volt_ka_unit_t = units::unit_t<turn_volt_ka_unit>;

using turn_volt_kv_unit =
    units::compound_unit<units::volts, units::inverse<radial_velocity>>;
using turn_volt_kv_unit_t = units::unit_t<turn_volt_kv_unit>;

using turn_volt_kp_unit =
    units::compound_unit<units::volts, units::inverse<units::turns>>;
using turn_volt_kp_unit_t = units::unit_t<turn_volt_kp_unit>;

using turn_volt_ki_unit = units::compound_unit<
    units::volts,
    units::inverse<units::compound_unit<units::turns, units::seconds>>>;
using turn_volt_ki_unit_t = units::unit_t<turn_volt_ki_unit>;

using turn_volt_kd_unit =
    units::compound_unit<units::volts,
                         units::inverse<units::turns_per_second>>;
using turn_volt_kd_unit_t = units::unit_t<turn_volt_kd_unit>;

using turn_amp_ka_unit =
    units::compound_unit<units::amperes, units::inverse<radial_accel>>;
using turn_amp_ka_unit_t = units::unit_t<turn_amp_ka_unit>;

using turn_amp_kv_unit =
    units::compound_unit<units::amperes, units::inverse<radial_velocity>>;
using turn_amp_kv_unit_t = units::unit_t<turn_amp_kv_unit>;

using turn_amp_kp_unit =
    units::compound_unit<units::amperes, units::inverse<units::turns>>;
using turn_amp_kp_unit_t = units::unit_t<turn_amp_kp_unit>;

using turn_amp_ki_unit = units::compound_unit<
    units::amperes,
    units::inverse<units::compound_unit<units::turns, units::seconds>>>;
using turn_amp_ki_unit_t = units::unit_t<turn_amp_ki_unit>;

using turn_amp_kd_unit =
    units::compound_unit<units::amperes,
                         units::inverse<units::turns_per_second>>;
using turn_amp_kd_unit_t = units::unit_t<turn_amp_kd_unit>;
} // namespace radial
} // namespace gains
} // namespace str
