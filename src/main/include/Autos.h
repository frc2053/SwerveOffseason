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
#include "choreo/auto/AutoFactory.h"
#include "choreo/auto/AutoLoop.h"
#include "choreo/auto/AutoTrajectory.h"

class Autos {
 public:
  frc2::CommandPtr TestChoreoAuto() {
    factory.Bind("test", [] { return frc2::cmd::Print("Hello from marker"); });
    straightTraj = factory.Trajectory("ChoreoTest", loop);

    loop.Enabled().OnTrue(frc2::cmd::RunOnce([this] {
                      swerveSub.ResetPose(
                          straightTraj.GetInitialPose().value());
                    })
                    .AndThen(straightTraj.Cmd())
                    .WithName("Straight Traj Name"));
    return loop.Cmd().WithName("Test Auto Loop Cmd");
  }


  explicit Autos(SwerveSubsystem& swerveSub, ShooterSubsystem& shooterSub,
                 IntakeSubsystem& intakeSub, FeederSubsystem& feederSub)
      : swerveSub{swerveSub},
        shooterSub{shooterSub},
        intakeSub{intakeSub},
        feederSub{feederSub},
        factory{swerveSub.GetFactory()},
        loop{factory.NewLoop("Auto Routine Loops")} {
    pathplanner::NamedCommands::registerCommand(
        "Print", frc2::cmd::Print("Test Named Command"));

    selectCommand = frc2::cmd::Select<AutoSelector>(
        [this] { return autoChooser.GetSelected(); },
        std::pair{CHOREO_TEST, TestChoreoAuto()},
        std::pair{PP_TEST, pathplanner::PathPlannerAuto("PPTest").ToPtr()});

    autoChooser.AddOption("Choreo Test", AutoSelector::CHOREO_TEST);
    autoChooser.AddOption("Path Planner Test", AutoSelector::PP_TEST);

    frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);
  }

  frc2::Command* GetSelectedCommand() { return selectCommand.get(); }

 private:
  enum AutoSelector { CHOREO_TEST, PP_TEST };

  frc::SendableChooser<AutoSelector> autoChooser;

  SwerveSubsystem& swerveSub;
  ShooterSubsystem& shooterSub;
  IntakeSubsystem& intakeSub;
  FeederSubsystem& feederSub;

  choreo::AutoTrajectory<choreo::SwerveSample> straightTraj;
  choreo::AutoFactory<choreo::SwerveSample>& factory;
  choreo::AutoLoop<choreo::SwerveSample> loop;

  frc2::CommandPtr selectCommand{frc2::cmd::None()};
};
