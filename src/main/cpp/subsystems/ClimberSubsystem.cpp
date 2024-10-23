// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "subsystems/ClimberSubsystem.h"

#include <frc2/command/Commands.h>

#include "frc/smartdashboard/SmartDashboard.h"
#include "wpi/sendable/SendableBuilder.h"
#include <frc/DataLogManager.h>

ClimberSubsystem::ClimberSubsystem() {
  ConfigureMotor();
}

void ClimberSubsystem::ConfigureMotor() {
  ctre::phoenix6::configs::TalonFXConfiguration climberConfig{};

  climberConfig.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Brake;
 
  ctre::phoenix::StatusCode leftclimberConfigResult =
      leftClimberMotor.GetConfigurator().Apply(climberConfig);

  frc::DataLogManager::Log(
      fmt::format("Configured left climber motor. Result was: {}\n",
                  leftclimberConfigResult.GetName()));

  ctre::phoenix::StatusCode rightClimberConfigResult =
      rightClimberMotor.GetConfigurator().Apply(climberConfig);

  frc::DataLogManager::Log(
      fmt::format("Configured right climber motor. Result was: {}\n",
                  rightClimberConfigResult.GetName()));
}

frc2::CommandPtr ClimberSubsystem::ManualControl(
    std::function<double()> left, std::function<double()> right) {
  return frc2::cmd::RunEnd(
      [this, left, right] {
        leftClimberMotor.Set(left());
        rightClimberMotor.Set(right());
      },
      [this] {
        leftClimberMotor.Set(0);
        rightClimberMotor.Set(0);
      },
      {this});
}

// This method will be called once per scheduler run
void ClimberSubsystem::Periodic() {

}

void ClimberSubsystem::SimulationPeriodic() {

}