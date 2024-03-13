#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include "str/Gains.h"

namespace str {

struct SwerveModuleConstants {
  const std::string moduleName;
  const int steerId;
  const int driveId;
  const int encoderId;
  const double steerEncoderOffset;
  const bool invertDrive;
  const bool invertSteer;
};

struct SwerveModulePhysical {
  const units::scalar_t steerGearing;
  const units::scalar_t driveGearing;
  const units::ampere_t supplySideLimit;
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
};

class SwerveModule {
public:
  explicit SwerveModule(SwerveModuleConstants constants, SwerveModulePhysical physicalAttrib, SwerveModuleSteerGains steerGains);
private:
  bool ConfigureSteerMotor(bool invertSteer, units::scalar_t steerGearing, units::ampere_t supplyCurrentLimit);
  bool ConfigureDriveMotor(bool invertDrive);
  bool ConfigureSteerEncoder(double encoderOffset);

  ctre::phoenix6::hardware::TalonFX steerMotor;
  ctre::phoenix6::hardware::TalonFX driveMotor;
  ctre::phoenix6::hardware::CANcoder steerEncoder;

  std::string moduleName;

  SwerveModuleSteerGains steerGains;
};
}
