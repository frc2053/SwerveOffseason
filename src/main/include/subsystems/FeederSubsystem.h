// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "frc/DigitalInput.h"
#include "networktables/BooleanTopic.h"
#include <constants/FeederConstants.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/filter/Debouncer.h>
#include <frc/simulation/FlywheelSim.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc2/command/SubsystemBase.h>
#include <networktables/BooleanTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <frc2/command/button/Trigger.h>

class FeederSubsystem : public frc2::SubsystemBase {
public:
  FeederSubsystem();


  frc2::CommandPtr Feed();
  frc2::CommandPtr Stop();
  frc2::CommandPtr Eject();

  frc2::Trigger GotNote() { return gotNote; }

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SimulationPeriodic() override;
  bool HasNote() const;

private:
  bool ConfigureFeederMotor(bool invert, units::scalar_t intakeGearing,
                            units::ampere_t supplyCurrentLimit,
                            units::ampere_t statorCurrentLimit);
  bool ConfigureMotorSignals();
  void UpdateNTEntries();

  ctre::phoenix6::hardware::TalonFX feederMotor{consts::feeder::can_ids::FEEDER,
                                                "rio"};

  frc::DigitalInput noteSensor{consts::feeder::ports::NOTE_SENSOR_PORT};
  frc::Debouncer noteSensorDebouncer{
      consts::feeder::gains::NOTE_SENSOR_DEBOUNCE_TIME,
      frc::Debouncer::DebounceType::kBoth};

  ctre::phoenix6::StatusSignal<units::volt_t> feederMotorVoltageSig =
      feederMotor.GetMotorVoltage();
  ctre::phoenix6::controls::VoltageOut feederMotorVoltageSetter{0_V};

  ctre::phoenix6::sim::TalonFXSimState &feederMotorSim =
      feederMotor.GetSimState();

  frc::LinearSystem<1, 1, 1> feederPlant{frc::LinearSystemId::FlywheelSystem(
      consts::feeder::physical::FEEDER_MOTOR,
      consts::feeder::physical::FEEDER_MOI,
      consts::feeder::physical::FEEDER_RATIO)};

  frc::sim::FlywheelSim feederSim{feederPlant,
                                  consts::feeder::physical::FEEDER_MOTOR};

  std::shared_ptr<nt::NetworkTable> nt{
      nt::NetworkTableInstance::GetDefault().GetTable("Feeder")};
  nt::DoublePublisher feederWheelMotorVoltagePub{
      nt->GetDoubleTopic("FeederWheelMotorVoltage").Publish()};
  nt::DoublePublisher feederWheelMotorVoltageSetpointPub{
      nt->GetDoubleTopic("FeederWheelMotorVoltageSetpoint").Publish()};
  nt::BooleanPublisher hasNotePub{nt->GetBooleanTopic("HasNote").Publish()};
  nt::BooleanPublisher noteSensorRawValPub{
      nt->GetBooleanTopic("NoteSensorRaw").Publish()};
  nt::BooleanPublisher noteSensorDeboucerPub{
      nt->GetBooleanTopic("NoteSensorDebouncer").Publish()};

  bool noteSensorRawVal{false};
  bool noteSensorDebouced{false};
  bool hasNote{false};

  frc2::Trigger gotNote{[this] { return hasNote; }};

  units::volt_t feederWheelVoltageSetpoint{0_V};
  units::volt_t currentFeederWheelVoltage{0_V};
};
