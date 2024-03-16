#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include "str/Gains.h"
#include <frc/kinematics/SwerveModuleState.h>
#include <networktables/StructTopic.h>
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
  void GoToState(frc::SwerveModuleState desiredState);
  frc::SwerveModulePosition GetCurrentPosition(bool refresh);
  frc::SwerveModuleState GetCurrentState();
  void UpdateSimulation(units::second_t deltaTime, units::volt_t supplyVoltage);
  std::array<ctre::phoenix6::BaseStatusSignal*, 6> GetSignals();
  bool OptimizeBusSignals();
  std::string GetName() const;
  units::ampere_t GetSimulatedCurrentDraw() const;
  void SetSteerToTorque(units::volt_t voltsToSend);
  void LogSteerTorqueSysId(frc::sysid::SysIdRoutineLog* log);
private:
  bool ConfigureSteerMotor(bool invertSteer, units::scalar_t steerGearing, units::ampere_t supplyCurrentLimit);
  bool ConfigureDriveMotor(bool invertDrive, units::ampere_t supplyCurrentLimit, units::ampere_t slipCurrentLimit);
  bool ConfigureSteerEncoder(double encoderOffset);
  void ConfigureControlSignals();
  units::turn_t ConvertDriveMotorRotationsToWheelRotations(units::turn_t motorRotations) const;
  units::turns_per_second_t ConvertDriveMotorVelToWheelVel(units::turns_per_second_t motorVel) const;
  units::meter_t ConvertWheelRotationsToWheelDistance(units::turn_t wheelRotations) const;
  units::meters_per_second_t ConvertWheelVelToLinearVel(units::turns_per_second_t wheelVel) const;
  units::turns_per_second_t ConvertLinearVelToWheelVel(units::meters_per_second_t linVel) const;
  units::turns_per_second_t ConvertWheelVelToMotorVel(units::turns_per_second_t wheelVel) const;

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

  ctre::phoenix6::controls::MotionMagicExpoTorqueCurrentFOC steerAngleSetter{0_rad};
  ctre::phoenix6::controls::VelocityTorqueCurrentFOC driveVelocitySetter{0_rad_per_s};

  //For characterization
  ctre::phoenix6::controls::TorqueCurrentFOC steerTorqueSetter{0_A};
  ctre::phoenix6::controls::TorqueCurrentFOC driveTorqueSetter{0_A};

  std::string moduleName;

  SwerveModuleSteerGains steerGains;
  SwerveModuleDriveGains driveGains;

  units::scalar_t couplingRatio;
  units::scalar_t driveGearing;
  units::meter_t wheelRadius;

  SwerveModuleSim moduleSim;

  std::shared_ptr<nt::NetworkTable> nt{nt::NetworkTableInstance::GetDefault().GetTable(moduleName + "_SwerveModule")};
  nt::StructTopic<frc::SwerveModuleState> desiredStateTopic{nt->GetStructTopic<frc::SwerveModuleState>("DesiredState")};
  nt::StructPublisher<frc::SwerveModuleState> desiredStatePub{desiredStateTopic.Publish()};
  
  nt::StructTopic<frc::SwerveModuleState> currentStateTopic{nt->GetStructTopic<frc::SwerveModuleState>("CurrentState")};
  nt::StructPublisher<frc::SwerveModuleState> currentStatePub{currentStateTopic.Publish()};

  nt::StructTopic<frc::SwerveModulePosition> currentPositionTopic{nt->GetStructTopic<frc::SwerveModulePosition>("CurrentPosition")};
  nt::StructPublisher<frc::SwerveModulePosition> currentPositionPub{currentPositionTopic.Publish()};
};
}
