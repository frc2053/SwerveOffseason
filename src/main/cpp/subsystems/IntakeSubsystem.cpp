// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "subsystems/IntakeSubsystem.h"

#include <frc/DataLogManager.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>

#include "constants/Constants.h"

IntakeSubsystem::IntakeSubsystem() {
  SetName("IntakeSubsystem");
  ConfigureIntakeMotor(consts::intake::physical::INVERT_MOTOR,
                       consts::intake::physical::INTAKE_RATIO,
                       consts::intake::current_limits::SUPPLY_CURRENT_LIMIT,
                       consts::intake::current_limits::STATOR_CURRENT_LIMIT);
  ConfigureMotorSignals();
  frc::SmartDashboard::PutData(this);
}

bool IntakeSubsystem::TouchingNote() {
  return isTouchingNote;
}

frc2::CommandPtr IntakeSubsystem::IntakeNote() {
  return frc2::cmd::RunEnd(
             [this] {
               intakeWheelVoltageSetpoint =
                   consts::intake::gains::NOTE_INTAKE_VOLTAGE;
             },
             [this] {
               intakeWheelVoltageSetpoint = 0_V;
               stateTimer.Restart();
             },
             {this})
      .WithName("IntakeNote")
      .BeforeStarting([this] { stateTimer.Restart(); });
}

frc2::CommandPtr IntakeSubsystem::Stop() {
  return frc2::cmd::RunOnce(
             [this] {
               intakeWheelVoltageSetpoint = 0_V;
               stateTimer.Restart();
             },
             {this})
      .WithName("Stop");
}

frc2::CommandPtr IntakeSubsystem::FakeNote() {
  return frc2::cmd::Run([this] { simOverrideTorque = true; }).FinallyDo([this] {
    simOverrideTorque = false;
  });
}

frc2::CommandPtr IntakeSubsystem::PoopNote() {
  return frc2::cmd::RunEnd(
      [this] {
        intakeWheelVoltageSetpoint = consts::intake::gains::NOTE_EJECT_VOLTAGE;
        stateTimer.Restart();
      },
      [this] {
        intakeWheelVoltageSetpoint = 0_V;
        stateTimer.Restart();
      },
      {this});
}

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic() {
  ctre::phoenix::StatusCode shooterWaitResult =
      ctre::phoenix6::BaseStatusSignal::RefreshAll(
          {&intakeMotorVoltageSig, &intakeMotorTorqueCurrentSig});

  currentIntakeWheelVoltage = intakeMotorVoltageSig.GetValue();
  if (simOverrideTorque) {
    intakeWheelTorqueCurrent = 300_A;
  } else {
    intakeWheelTorqueCurrent = intakeMotorTorqueCurrentSig.GetValue();
  }

  isTouchingNote = stateTimer.HasElapsed(.25_s) &&
                   intakeSpikeDebouncer.Calculate(
                       intakeWheelTorqueCurrent >
                       consts::intake::gains::NOTE_SPIKE_THRESHOLD);

  intakeMotor.SetControl(
      intakeMotorVoltageSetter.WithOutput(intakeWheelVoltageSetpoint));

  UpdateNTEntries();
}

void IntakeSubsystem::UpdateNTEntries() {
  intakeWheelMotorVoltagePub.Set(currentIntakeWheelVoltage.value());
  intakeWheelMotorVoltageSetpointPub.Set(intakeWheelVoltageSetpoint.value());
  intakeWheelTorqueCurrentPub.Set(intakeWheelTorqueCurrent.value());
  touchingNotePub.Set(isTouchingNote);
}

void IntakeSubsystem::SimulationPeriodic() {
  intakeMotorSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
  intakeSim.SetInputVoltage(intakeMotorSim.GetMotorVoltage());
  intakeSim.Update(consts::LOOP_PERIOD);
  intakeMotorSim.SetRotorVelocity(intakeSim.GetAngularVelocity() *
                                  consts::intake::physical::INTAKE_RATIO);
}

bool IntakeSubsystem::ConfigureIntakeMotor(bool invert,
                                           units::scalar_t intakeGearing,
                                           units::ampere_t supplyCurrentLimit,
                                           units::ampere_t statorCurrentLimit) {
  ctre::phoenix6::configs::TalonFXConfiguration intakeConfig{};

  intakeConfig.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Brake;
  intakeConfig.CurrentLimits.StatorCurrentLimit = statorCurrentLimit.value();
  intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;

  intakeConfig.MotorOutput.Inverted =
      invert
          ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
          : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;

  intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  intakeConfig.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit.value();

  ctre::phoenix::StatusCode intakeConfigResult =
      intakeMotor.GetConfigurator().Apply(intakeConfig);

  frc::DataLogManager::Log(
      fmt::format("Configured intake motor. Result was: {}\n",
                  intakeConfigResult.GetName()));

  return intakeConfigResult.IsOK();
}

bool IntakeSubsystem::ConfigureMotorSignals() {
  intakeMotorVoltageSetter.UpdateFreqHz = 0_Hz;

  // Double the rio update rate? Not sure what is optimal here
  units::hertz_t updateRate = 1.0 / (consts::LOOP_PERIOD * 2.0);

  intakeMotorVoltageSig.SetUpdateFrequency(updateRate);
  intakeMotorTorqueCurrentSig.SetUpdateFrequency(updateRate);

  ctre::phoenix::StatusCode optimizeIntakeMotor =
      intakeMotor.OptimizeBusUtilization();
  if (optimizeIntakeMotor.IsOK()) {
    frc::DataLogManager::Log("Optimized bus signals for intake motor\n");
  }

  return optimizeIntakeMotor.IsOK();
}
