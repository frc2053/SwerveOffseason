// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/filter/SlewRateLimiter.h>
#include <frc/system/plant/DCMotor.h>
#include <str/Gains.h>
#include <str/Units.h>
#include <units/base.h>

#include <string>

namespace str {
struct SwerveModuleConstants {
  const std::string moduleName;
  const int steerId;
  const int driveId;
  const int encoderId;
  const units::turn_t steerEncoderOffset;
  const bool invertDrive;
  const bool invertSteer;
};

struct SwerveModulePhysical {
  const units::scalar_t steerGearing;
  const units::scalar_t driveGearing;
  const units::ampere_t driveSupplySideLimit;
  const units::ampere_t steerTorqueCurrentLimit;
  const units::ampere_t slipCurrent;
  const frc::DCMotor driveMotor;
  const frc::DCMotor steerMotor;
  const units::scalar_t couplingRatio;
  const units::meter_t wheelRadius;
  // Used for sim only
  const units::volt_t driveFrictionVoltage;
  const units::volt_t steerFrictionVoltage;
};

struct SwerveModuleSteerGains {
  units::turns_per_second_t motionMagicCruiseVel;
  str::gains::radial::turn_volt_ka_unit_t motionMagicExpoKa;
  str::gains::radial::turn_volt_kv_unit_t motionMagicExpoKv;
  str::gains::radial::turn_amp_ka_unit_t kA;
  str::gains::radial::turn_amp_kv_unit_t kV;
  units::ampere_t kS;
  str::gains::radial::turn_amp_kp_unit_t kP;
  str::gains::radial::turn_amp_ki_unit_t kI;
  str::gains::radial::turn_amp_kd_unit_t kD;

  bool operator==(const SwerveModuleSteerGains& rhs) const {
    return units::essentiallyEqual(kA, rhs.kA, 1e-6) &&
           units::essentiallyEqual(kV, rhs.kV, 1e-6) &&
           units::essentiallyEqual(kS, rhs.kS, 1e-6) &&
           units::essentiallyEqual(kP, rhs.kP, 1e-6) &&
           units::essentiallyEqual(kI, rhs.kI, 1e-6) &&
           units::essentiallyEqual(kD, rhs.kD, 1e-6);
  }
  bool operator!=(const SwerveModuleSteerGains& rhs) const {
    return !operator==(rhs);
  }
};

struct SwerveModuleDriveGains {
  str::gains::radial::turn_amp_ka_unit_t kA;
  str::gains::radial::turn_amp_kv_unit_t kV;
  units::ampere_t kS;
  str::gains::radial::turn_amp_kp_unit_t kP;
  str::gains::radial::turn_amp_ki_unit_t kI;
  str::gains::radial::turn_amp_kd_unit_t kD;

  bool operator==(const SwerveModuleDriveGains& rhs) const {
    return units::essentiallyEqual(kA, rhs.kA, 1e-6) &&
           units::essentiallyEqual(kV, rhs.kV, 1e-6) &&
           units::essentiallyEqual(kS, rhs.kS, 1e-6) &&
           units::essentiallyEqual(kP, rhs.kP, 1e-6) &&
           units::essentiallyEqual(kI, rhs.kI, 1e-6) &&
           units::essentiallyEqual(kD, rhs.kD, 1e-6);
  }
  bool operator!=(const SwerveModuleDriveGains& rhs) const {
    return !operator==(rhs);
  }
};

struct WheelRadiusCharData {
  units::radian_t lastGyroYaw;
  units::radian_t accumGyroYaw;
  std::array<units::radian_t, 4> startWheelPositions;
  units::meter_t effectiveWheelRadius = 0_m;
  frc::SlewRateLimiter<units::radians_per_second> omegaLimiter{1_rad_per_s /
                                                               1_s};
};
}  // namespace str
