#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include "str/Gains.h"
#include <frc/kinematics/SwerveModuleState.h>
#include <networktables/StructTopic.h>
#include <frc/system/plant/DCMotor.h>
#include "str/SwerveModuleSim.h"
#include "str/SwerveModuleHelpers.h"

namespace str {

class SwerveModule {
public:
  explicit SwerveModule(SwerveModuleConstants constants, SwerveModulePhysical physicalAttrib, SwerveModuleSteerGains steerGains, SwerveModuleDriveGains driveGains);
  void GoToState(frc::SwerveModuleState desiredState);
  frc::SwerveModuleState GetCurrentState() const;
private:
  bool ConfigureSteerMotor(bool invertSteer, units::scalar_t steerGearing, units::ampere_t supplyCurrentLimit);
  bool ConfigureDriveMotor(bool invertDrive, units::ampere_t supplyCurrentLimit, units::ampere_t slipCurrentLimit );
  bool ConfigureSteerEncoder(double encoderOffset);

  ctre::phoenix6::hardware::TalonFX steerMotor;
  ctre::phoenix6::hardware::TalonFX driveMotor;
  ctre::phoenix6::hardware::CANcoder steerEncoder;

  std::string moduleName;

  SwerveModuleSteerGains steerGains;
  SwerveModuleDriveGains driveGains;

  SwerveModuleSim moduleSim;

  std::shared_ptr<nt::NetworkTable> nt;
  nt::StructTopic<frc::SwerveModuleState> desiredStateTopic;
  nt::StructPublisher<frc::SwerveModuleState> desiredStatePub;
};
}
