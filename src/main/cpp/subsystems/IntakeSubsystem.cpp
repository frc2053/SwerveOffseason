// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "constants/Constants.h"

IntakeSubsystem::IntakeSubsystem() {
    SetName("IntakeSubsystem");
    ConfigureIntakeMotor(consts::intake::physical::INVERT_MOTOR, consts::intake::physical::INTAKE_RATIO, consts::intake::current_limits::SUPPLY_CURRENT_LIMIT, consts::intake::current_limits::STATOR_CURRENT_LIMIT);
    ConfigureMotorSignals();
    frc::SmartDashboard::PutData(this);
}

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic() {}

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