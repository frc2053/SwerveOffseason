// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {

  swerveSubsystem.SetDefaultCommand(swerveSubsystem.DriveModule(
  [this] {
    return units::radian_t{std::atan2(-controller.GetLeftY(), -controller.GetLeftX())};
  }, 
  [this] {
    return units::feet_per_second_t{std::hypot(-controller.GetLeftY(), -controller.GetLeftX())} * 12;
  }));

  controller.A().WhileTrue(swerveSubsystem.SysIdSteerQuasistatic(frc2::sysid::Direction::kForward));
  controller.B().WhileTrue(swerveSubsystem.SysIdSteerQuasistatic(frc2::sysid::Direction::kReverse));
  controller.X().WhileTrue(swerveSubsystem.SysIdSteerDynamic(frc2::sysid::Direction::kForward));
  controller.Y().WhileTrue(swerveSubsystem.SysIdSteerDynamic(frc2::sysid::Direction::kReverse));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}

SwerveSubsystem& RobotContainer::GetSwerveSubsystem() {
  return swerveSubsystem;
}
