#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include "str/Gains.h"
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/system/plant/DCMotor.h>
#include "str/SwerveModuleSim.h"
#include "str/SwerveModuleHelpers.h"
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <ctre/phoenix6/SignalLogger.hpp>

namespace str {

class SwerveModule {
public:
  explicit SwerveModule(SwerveModuleConstants constants, SwerveModulePhysical physicalAttrib, SwerveModuleSteerGains steerGains, SwerveModuleDriveGains driveGains);
  // We return the optimized state to make sure we log what we actually set the module to
  frc::SwerveModuleState GoToState(frc::SwerveModuleState desiredState, bool optimize=true);
  frc::SwerveModulePosition GetCurrentPosition(bool refresh);
  units::radian_t GetOutputShaftTurns();
  frc::SwerveModuleState GetCurrentState();
  frc::SwerveModuleState UpdateSimulation(units::second_t deltaTime, units::volt_t supplyVoltage);
  std::array<ctre::phoenix6::BaseStatusSignal*, 8> GetSignals();
  bool OptimizeBusSignals();
  std::string GetName() const;
  units::ampere_t GetSimulatedCurrentDraw() const;
  void SetSteerToTorque(units::volt_t voltsToSend);
  void SetDriveToTorque(units::volt_t voltsToSend);
  void SetSteerToVoltage(units::volt_t voltsToSend);
  void SetDriveToVoltage(units::volt_t voltsToSend);
private:
  bool ConfigureSteerMotor(bool invertSteer, units::scalar_t steerGearing, units::ampere_t supplyCurrentLimit);
  bool ConfigureDriveMotor(bool invertDrive, units::ampere_t supplyCurrentLimit, units::ampere_t slipCurrentLimit);
  bool ConfigureSteerEncoder(units::turn_t encoderOffset);
  void ConfigureControlSignals();
  units::radian_t ConvertDriveMotorRotationsToWheelRotations(units::radian_t motorRotations) const;
  units::radians_per_second_t ConvertDriveMotorVelToWheelVel(units::radians_per_second_t motorVel) const;
  units::meter_t ConvertWheelRotationsToWheelDistance(units::radian_t wheelRotations) const;
  units::meters_per_second_t ConvertWheelVelToLinearVel(units::radians_per_second_t wheelVel) const;
  units::radians_per_second_t ConvertLinearVelToWheelVel(units::meters_per_second_t linVel) const;
  units::radians_per_second_t ConvertWheelVelToMotorVel(units::radians_per_second_t wheelVel) const;

  ctre::phoenix6::hardware::TalonFX steerMotor;
  ctre::phoenix6::hardware::TalonFX driveMotor;
  ctre::phoenix6::hardware::CANcoder steerEncoder;

  ctre::phoenix6::StatusSignal<units::turn_t> drivePositionSig = driveMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> driveVelocitySig = driveMotor.GetVelocity();
  ctre::phoenix6::StatusSignal<units::turn_t> steerPositionSig = steerMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> steerVelocitySig = steerMotor.GetVelocity();

  //For characterization
  ctre::phoenix6::StatusSignal<units::ampere_t> driveTorqueCurrentSig = driveMotor.GetTorqueCurrent();
  ctre::phoenix6::StatusSignal<units::ampere_t> steerTorqueCurrentSig = steerMotor.GetTorqueCurrent();
  ctre::phoenix6::StatusSignal<units::volt_t> driveVoltageSig = driveMotor.GetMotorVoltage();
  ctre::phoenix6::StatusSignal<units::volt_t> steerVoltageSig = steerMotor.GetMotorVoltage();

  ctre::phoenix6::controls::MotionMagicExpoTorqueCurrentFOC steerAngleSetter{0_rad};
  ctre::phoenix6::controls::VelocityTorqueCurrentFOC driveVelocitySetter{0_rad_per_s};

  //For characterization
  ctre::phoenix6::controls::TorqueCurrentFOC steerTorqueSetter{0_A};
  ctre::phoenix6::controls::TorqueCurrentFOC driveTorqueSetter{0_A};
  ctre::phoenix6::controls::VoltageOut steerVoltageSetter{0_V};
  ctre::phoenix6::controls::VoltageOut driveVoltageSetter{0_V};

  std::string moduleName;

  SwerveModuleSteerGains steerGains;
  SwerveModuleDriveGains driveGains;

  units::scalar_t couplingRatio;
  units::scalar_t driveGearing;
  units::meter_t wheelRadius;

  SwerveModuleSim moduleSim;
};
}
