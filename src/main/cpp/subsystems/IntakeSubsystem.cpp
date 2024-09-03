// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "constants/Constants.h"
#include <frc2/command/Commands.h>

IntakeSubsystem::IntakeSubsystem() {
    SetName("IntakeSubsystem");
    ConfigureIntakeMotor(consts::intake::physical::INVERT_MOTOR, consts::intake::physical::INTAKE_RATIO, consts::intake::current_limits::SUPPLY_CURRENT_LIMIT, consts::intake::current_limits::STATOR_CURRENT_LIMIT);
    ConfigureMotorSignals();
    frc::SmartDashboard::PutData(this);
}

bool IntakeSubsystem::TouchingNote() {
  return isTouchingNote;
}

frc2::CommandPtr IntakeSubsystem::IntakeNote() {
  return frc2::cmd::RunEnd([this] {
    intakeWheelVoltageSetpoint = consts::intake::gains::NOTE_INTAKE_VOLTAGE;
  }, [this] {
    intakeWheelVoltageSetpoint = 0_V;
  }, {this});
}

frc2::CommandPtr IntakeSubsystem::PoopNote() {
  return frc2::cmd::RunEnd([this] {
    intakeWheelVoltageSetpoint = consts::intake::gains::NOTE_EJECT_VOLTAGE;
  }, [this] {
    intakeWheelVoltageSetpoint = 0_V;
  }, {this});
}

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic() {
  ctre::phoenix::StatusCode shooterWaitResult = ctre::phoenix6::BaseStatusSignal::RefreshAll({
    &intakeMotorVoltageSig, 
    &intakeMotorTorqueCurrentSig
  });

  currentIntakeWheelVoltage = intakeMotorVoltageSig.GetValue();
  intakeWheelTorqueCurrent = intakeMotorTorqueCurrentSig.GetValue();

  isTouchingNote = intakeSpikeDebouncer.Calculate(intakeWheelTorqueCurrent > consts::intake::gains::NOTE_SPIKE_THRESHOLD);

  intakeMotor.SetControl(intakeMotorVoltageSetter.WithOutput(intakeWheelVoltageSetpoint));

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
    intakeMotorSim.SetRotorVelocity(intakeSim.GetAngularVelocity() * consts::intake::physical::INTAKE_RATIO);
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

  fmt::print("Configured intake motor. Result was: {}\n",
             intakeConfigResult.GetName());

  return intakeConfigResult.IsOK();
}


bool IntakeSubsystem::ConfigureMotorSignals() {
  intakeMotorVoltageSetter.UpdateFreqHz = 0_Hz;

  //Double the rio update rate? Not sure what is optimal here
  units::hertz_t updateRate = 1.0 / (consts::LOOP_PERIOD * 2.0);

  intakeMotorVoltageSig.SetUpdateFrequency(updateRate);
  intakeMotorTorqueCurrentSig.SetUpdateFrequency(updateRate);

  ctre::phoenix::StatusCode optimizeIntakeMotor =
      intakeMotor.OptimizeBusUtilization();
  if (optimizeIntakeMotor.IsOK()) {
    fmt::print("Optimized bus signals for intake motor\n");
  }

  return optimizeIntakeMotor.IsOK();
}