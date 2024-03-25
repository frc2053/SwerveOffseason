#pragma once

#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Commands.h>

class Autos {
public:
  Autos() {
    autoChooser.SetDefaultOption("Drive Forward", AutoSelector::DRIVE_FORWARD);

    frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);
  };

  frc2::Command* GetSelectedCommand() {
    return selectCommand.get();
  }
private:
  enum AutoSelector { DRIVE_FORWARD };

  frc::SendableChooser<AutoSelector> autoChooser;

  frc2::CommandPtr selectCommand = frc2::cmd::Select<AutoSelector>(
    [this] { return autoChooser.GetSelected(); },
    std::pair{DRIVE_FORWARD, pathplanner::PathPlannerAuto("DriveForward").ToPtr()}
  );
};