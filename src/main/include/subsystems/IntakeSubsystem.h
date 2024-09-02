// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <units/current.h>
#include <units/base.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleTopic.h>
#include <constants/IntakeConstants.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/simulation/FlywheelSim.h>

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;


 private:
  void UpdateNTEntries();
  bool ConfigureIntakeMotor(bool invert,
                              units::scalar_t intakeGearing,
                              units::ampere_t supplyCurrentLimit,
                              units::ampere_t statorCurrentLimit);
  bool ConfigureMotorSignals();

  ctre::phoenix6::hardware::TalonFX intakeMotor{consts::intake::can_ids::INTAKE, "*"};
  ctre::phoenix6::StatusSignal<units::volt_t> intakeMotorVoltageSig = intakeMotor.GetMotorVoltage();
  ctre::phoenix6::StatusSignal<units::ampere_t> intakeMotorTorqueCurrentSig = intakeMotor.GetTorqueCurrent();
  ctre::phoenix6::controls::VoltageOut intakeMotorVoltageSetter{0_V};

  ctre::phoenix6::sim::TalonFXSimState &intakeMotorSim = intakeMotor.GetSimState();

  frc::LinearSystem<1, 1, 1>  intakePlant{frc::LinearSystemId::FlywheelSystem(
      consts::intake::physical::INTAKE_MOTOR, consts::intake::physical::INTAKE_MOI, consts::intake::physical::INTAKE_RATIO)};

  frc::sim::FlywheelSim intakeSim{intakePlant, consts::intake::physical::INTAKE_MOTOR};

  std::shared_ptr<nt::NetworkTable> nt{
      nt::NetworkTableInstance::GetDefault().GetTable("Intake")};
};
