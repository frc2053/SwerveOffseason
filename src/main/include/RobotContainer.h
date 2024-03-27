// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include "subsystems/SwerveSubsystem.h"
#include "str/Vision.h"
#include <frc2/command/button/CommandXboxController.h>
#include "Autos.h"

class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

  SwerveSubsystem& GetSwerveSubsystem();
  str::Vision& GetVision();

 private:
  void ConfigureBindings();
  frc2::CommandXboxController controller{0};
  
  SwerveSubsystem swerveSubsystem;
  str::Vision vision;

  Autos autos;
};
