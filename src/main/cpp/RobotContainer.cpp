// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include <frc/MathUtil.h>

RobotContainer::RobotContainer() {
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {

  swerveSubsystem.SetDefaultCommand(swerveSubsystem.Drive([this] {
    return frc::ApplyDeadband<double>(-controller.GetLeftY(), .1) * consts::swerve::physical::DRIVE_MAX_SPEED;
  },
  [this] {
    return frc::ApplyDeadband<double>(-controller.GetLeftX(), .1) * consts::swerve::physical::DRIVE_MAX_SPEED;
  },
  [this] {
    return frc::ApplyDeadband<double>(-controller.GetRightX(), .1) * consts::swerve::physical::DRIVE_MAX_ROT_SPEED;
  },
  true
  ));

  // controller.A().WhileTrue(swerveSubsystem.SysIdDriveQuasistaticTorque(frc2::sysid::Direction::kForward));
  // controller.B().WhileTrue(swerveSubsystem.SysIdDriveQuasistaticTorque(frc2::sysid::Direction::kReverse));
  // controller.X().WhileTrue(swerveSubsystem.SysIdDriveDynamicTorque(frc2::sysid::Direction::kForward));
  // controller.Y().WhileTrue(swerveSubsystem.SysIdDriveDynamicTorque(frc2::sysid::Direction::kReverse));

  // controller.A().WhileTrue(swerveSubsystem.SysIdSteerQuasistaticTorque(frc2::sysid::Direction::kForward));
  // controller.B().WhileTrue(swerveSubsystem.SysIdSteerQuasistaticTorque(frc2::sysid::Direction::kReverse));
  // controller.X().WhileTrue(swerveSubsystem.SysIdSteerDynamicTorque(frc2::sysid::Direction::kForward));
  // controller.Y().WhileTrue(swerveSubsystem.SysIdSteerDynamicTorque(frc2::sysid::Direction::kReverse));

  // controller.A().WhileTrue(swerveSubsystem.SysIdSteerQuasistaticVoltage(frc2::sysid::Direction::kForward));
  // controller.B().WhileTrue(swerveSubsystem.SysIdSteerQuasistaticVoltage(frc2::sysid::Direction::kReverse));
  // controller.X().WhileTrue(swerveSubsystem.SysIdSteerDynamicVoltage(frc2::sysid::Direction::kForward));
  // controller.Y().WhileTrue(swerveSubsystem.SysIdSteerDynamicVoltage(frc2::sysid::Direction::kReverse));

  controller.A().WhileTrue(swerveSubsystem.SysIdDriveQuasistaticVoltage(frc2::sysid::Direction::kForward));
  controller.B().WhileTrue(swerveSubsystem.SysIdDriveQuasistaticVoltage(frc2::sysid::Direction::kReverse));
  controller.X().WhileTrue(swerveSubsystem.SysIdDriveDynamicVoltage(frc2::sysid::Direction::kForward));
  controller.Y().WhileTrue(swerveSubsystem.SysIdDriveDynamicVoltage(frc2::sysid::Direction::kReverse));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}

SwerveSubsystem& RobotContainer::GetSwerveSubsystem() {
  return swerveSubsystem;
}
