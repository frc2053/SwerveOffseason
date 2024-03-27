// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include <frc/MathUtil.h>
#include <str/DriverstationUtils.h>

RobotContainer::RobotContainer() {
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {

  swerveSubsystem.SetDefaultCommand(swerveSubsystem.Drive([this] {
    return str::NegateIfRed(frc::ApplyDeadband<double>(-controller.GetLeftY(), .1) * consts::swerve::physical::DRIVE_MAX_SPEED);
  },
  [this] {
    return str::NegateIfRed(frc::ApplyDeadband<double>(-controller.GetLeftX(), .1) * consts::swerve::physical::DRIVE_MAX_SPEED);
  },
  [this] {
    return frc::ApplyDeadband<double>(-controller.GetRightX(), .1) * consts::swerve::physical::DRIVE_MAX_ROT_SPEED;
  },
  true
  ));

  controller.LeftTrigger().WhileTrue(swerveSubsystem.AlignToAmp());
  controller.Back().WhileTrue(swerveSubsystem.WheelRadius(frc2::sysid::Direction::kReverse));
  controller.Start().WhileTrue(swerveSubsystem.WheelRadius(frc2::sysid::Direction::kForward));

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

  // controller.A().WhileTrue(swerveSubsystem.SysIdDriveQuasistaticVoltage(frc2::sysid::Direction::kForward));
  // controller.B().WhileTrue(swerveSubsystem.SysIdDriveQuasistaticVoltage(frc2::sysid::Direction::kReverse));
  // controller.X().WhileTrue(swerveSubsystem.SysIdDriveDynamicVoltage(frc2::sysid::Direction::kForward));
  // controller.Y().WhileTrue(swerveSubsystem.SysIdDriveDynamicVoltage(frc2::sysid::Direction::kReverse));

  // controller.A().WhileTrue(swerveSubsystem.PointWheelsToAngle([] { return 0_deg; }));
  // controller.B().WhileTrue(swerveSubsystem.PointWheelsToAngle([] { return 45_deg; }));
  // controller.X().WhileTrue(swerveSubsystem.PointWheelsToAngle([] { return 90_deg; }));
  // controller.Y().WhileTrue(swerveSubsystem.PointWheelsToAngle([] { return 180_deg; }));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return autos.GetSelectedCommand();
}

SwerveSubsystem& RobotContainer::GetSwerveSubsystem() {
  return swerveSubsystem;
}
