// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <constants/IntakeConstants.h>
#include <frc/filter/Debouncer.h>
#include <frc/simulation/FlywheelSim.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/button/Trigger.h>
#include <networktables/BooleanTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <units/base.h>
#include <units/current.h>

#include <memory>

#include <ctre/phoenix6/TalonFX.hpp>

class IntakeSubsystem : public frc2::SubsystemBase {
public:
  IntakeSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SimulationPeriodic() override;
  bool TouchingNote();
  frc2::Trigger TouchedNote() { return touchedNote; }

  frc2::CommandPtr IntakeNote();
  frc2::CommandPtr Stop();
  frc2::CommandPtr PoopNote();
  frc2::CommandPtr FakeNote();

private:
  void UpdateNTEntries();
  bool ConfigureIntakeMotor(bool invert, units::scalar_t intakeGearing,
                            units::ampere_t supplyCurrentLimit,
                            units::ampere_t statorCurrentLimit);
  bool ConfigureMotorSignals();

  ctre::phoenix6::hardware::TalonFX intakeMotor{consts::intake::can_ids::INTAKE,
                                                "rio"};
  ctre::phoenix6::StatusSignal<units::volt_t> intakeMotorVoltageSig =
      intakeMotor.GetMotorVoltage();
  ctre::phoenix6::StatusSignal<units::ampere_t> intakeMotorTorqueCurrentSig =
      intakeMotor.GetTorqueCurrent();
  ctre::phoenix6::controls::VoltageOut intakeMotorVoltageSetter{0_V};

  ctre::phoenix6::sim::TalonFXSimState &intakeMotorSim =
      intakeMotor.GetSimState();

  frc::LinearSystem<1, 1, 1> intakePlant{frc::LinearSystemId::FlywheelSystem(
      consts::intake::physical::INTAKE_MOTOR,
      consts::intake::physical::INTAKE_MOI,
      consts::intake::physical::INTAKE_RATIO)};

  frc::sim::FlywheelSim intakeSim{intakePlant,
                                  consts::intake::physical::INTAKE_MOTOR};

  std::shared_ptr<nt::NetworkTable> nt{
      nt::NetworkTableInstance::GetDefault().GetTable("Intake")};
  nt::DoublePublisher intakeWheelMotorVoltagePub{
      nt->GetDoubleTopic("IntakeWheelMotorVoltage").Publish()};
  nt::DoublePublisher intakeWheelMotorVoltageSetpointPub{
      nt->GetDoubleTopic("IntakeWheelMotorVoltageSetpoint").Publish()};
  nt::DoublePublisher intakeWheelTorqueCurrentPub{
      nt->GetDoubleTopic("IntakeWheelMotorTorqueCurrent").Publish()};
  nt::BooleanPublisher touchingNotePub{
      nt->GetBooleanTopic("TouchingNote").Publish()};

  frc::Timer stateTimer{};
  frc::Debouncer intakeSpikeDebouncer{.25_s, frc::Debouncer::kFalling};
  bool isTouchingNote{false};
  bool simOverrideTorque{false};

  frc2::Trigger touchedNote{[this] { return isTouchingNote; }};

  units::volt_t intakeWheelVoltageSetpoint{0_V};
  units::volt_t currentIntakeWheelVoltage{0_V};
  units::ampere_t intakeWheelTorqueCurrent{0_A};
};
