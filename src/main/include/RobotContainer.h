// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Autos.h"
#include "str/NoteVisualizer.h"
#include "str/Vision.h"
#include "subsystems/SwerveSubsystem.h"

class RobotContainer {
public:
  RobotContainer();

  frc2::Command *GetAutonomousCommand();

  SwerveSubsystem &GetSwerveSubsystem();
  ShooterSubsystem &GetShooterSubsystem();
  str::Vision &GetVision();
  str::NoteVisualizer &GetNoteVisualizer();

private:
  void ConfigureBindings();
  frc2::CommandXboxController controller{0};

  SwerveSubsystem swerveSubsystem;
  ShooterSubsystem shooterSubsystem;
  str::Vision vision;
  str::NoteVisualizer noteVisualizer;

  Autos autos{swerveSubsystem, shooterSubsystem};
};
