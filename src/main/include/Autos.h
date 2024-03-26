#pragma once

#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/NamedCommands.h>

class Autos {
public:
  Autos() {

    pathplanner::NamedCommands::registerCommand("Shoot", frc2::cmd::Print("Shooting Note"));

    selectCommand = frc2::cmd::Select<AutoSelector>(
      [this] { return autoChooser.GetSelected(); },
      std::pair{DRIVE_FORWARD, pathplanner::PathPlannerAuto("DriveForward").ToPtr()},
      std::pair{AMP_SIDE_FIVE, pathplanner::PathPlannerAuto("AmpSideFive").ToPtr()}
    );


    autoChooser.SetDefaultOption("Drive Forward", AutoSelector::DRIVE_FORWARD);
    autoChooser.AddOption("Amp Side Five", AutoSelector::AMP_SIDE_FIVE);

    frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);
  };

  frc2::Command* GetSelectedCommand() {
    return selectCommand.get();
  }
private:
  enum AutoSelector { DRIVE_FORWARD, AMP_SIDE_FIVE };

  frc::SendableChooser<AutoSelector> autoChooser;


  frc2::CommandPtr selectCommand{frc2::cmd::None()};
};