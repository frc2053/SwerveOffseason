// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <subsystems/FeederSubsystem.h>
#include <subsystems/IntakeSubsystem.h>
#include <subsystems/ShooterSubsystem.h>
#include <subsystems/SwerveSubsystem.h>

class Autos {
 public:
  explicit Autos(SwerveSubsystem& swerveSub, ShooterSubsystem& shooterSub,
                 IntakeSubsystem& intakeSub, FeederSubsystem& feederSub)
      : swerveSub(swerveSub),
        shooterSub(shooterSub),
        intakeSub(intakeSub),
        feederSub(feederSub) {
    pathplanner::NamedCommands::registerCommand(
        "Shoot", frc2::cmd::Print("Shooting Note"));

    selectCommand = frc2::cmd::Select<AutoSelector>(
        [this] { return autoChooser.GetSelected(); },
        std::pair{CHOREO_TEST,
                  swerveSub.FollowChoreoTrajectory([] { return "TestPath"; })},
        std::pair{CHOREO_LIME,
                  swerveSub.FollowChoreoTrajectory([] { return "Lime"; })});

    autoChooser.AddOption("Choreo Test", AutoSelector::CHOREO_TEST);
    autoChooser.AddOption("Choreo Lime", AutoSelector::CHOREO_LIME);

    frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);
  }

  frc2::Command* GetSelectedCommand() { return selectCommand.get(); }

 private:
  enum AutoSelector { CHOREO_TEST, CHOREO_LIME };

  frc::SendableChooser<AutoSelector> autoChooser;

  SwerveSubsystem& swerveSub;
  ShooterSubsystem& shooterSub;
  IntakeSubsystem& intakeSub;
  FeederSubsystem& feederSub;

  frc2::CommandPtr selectCommand{frc2::cmd::None()};
};
