// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <subsystems/ShooterSubsystem.h>
#include <subsystems/SwerveSubsystem.h>

class Autos {
public:
  explicit Autos(SwerveSubsystem &swerveSub, ShooterSubsystem &shooterSub)
      : swerveSub(swerveSub), shooterSub(shooterSub) {
    pathplanner::NamedCommands::registerCommand(
        "Shoot", frc2::cmd::Print("Shooting Note"));

    selectCommand = frc2::cmd::Select<AutoSelector>(
        [this] { return autoChooser.GetSelected(); },
        std::pair{DRIVE_FORWARD,
                  pathplanner::PathPlannerAuto("DriveForward").ToPtr()},
        std::pair{AMP_SIDE_FIVE,
                  pathplanner::PathPlannerAuto("AmpSideFive").ToPtr()},
        std::pair{CHOREO_TEST, swerveSub.FollowChoreoTrajectory(
                                   [] { return "AmpSideFive"; })},
        std::pair{CHOREO_SQUARE,
                  swerveSub.FollowChoreoTrajectory([] { return "Square"; })});

    autoChooser.SetDefaultOption("Drive Forward", AutoSelector::DRIVE_FORWARD);
    autoChooser.AddOption("Amp Side Five", AutoSelector::AMP_SIDE_FIVE);
    autoChooser.AddOption("Choreo Test", AutoSelector::CHOREO_TEST);
    autoChooser.AddOption("Choreo Square", AutoSelector::CHOREO_SQUARE);

    frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);
  }

  frc2::Command *GetSelectedCommand() { return selectCommand.get(); }

private:
  enum AutoSelector {
    DRIVE_FORWARD,
    AMP_SIDE_FIVE,
    CHOREO_TEST,
    CHOREO_SQUARE
  };

  frc::SendableChooser<AutoSelector> autoChooser;

  SwerveSubsystem &swerveSub;
  ShooterSubsystem &shooterSub;

  frc2::CommandPtr selectCommand{frc2::cmd::None()};
};
