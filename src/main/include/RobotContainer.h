// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc2/command/CommandPtr.h>
//#include <str/CommandLogitechController.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/NetworkButton.h>

#include <functional>

#include "Autos.h"
#include "str/NoteVisualizer.h"
#include "str/Vision.h"
#include "subsystems/FeederSubsystem.h"
#include "subsystems/SwerveSubsystem.h"
#include "subsystems/ClimberSubsystem.h"

class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

  frc2::CommandPtr IntakeNote();

  SwerveSubsystem& GetSwerveSubsystem();
  ShooterSubsystem& GetShooterSubsystem();
  IntakeSubsystem& GetIntakeSubsystem();
  FeederSubsystem& GetFeederSubsystem();
  str::Vision &GetVision();
  str::NoteVisualizer& GetNoteVisualizer();

 private:
  void ConfigureBindings();
  frc2::CommandXboxController driverController{0};
  frc2::CommandXboxController operatorController{1};
  //frc2::CommandLogitechController sylaceController{2};

  std::shared_ptr<nt::NetworkTable> tuningTable{nt::NetworkTableInstance::GetDefault().GetTable("Tuning")};

  frc2::NetworkButton wheelRadFwdBtn{tuningTable, "WheelRadiusFwd"};
  frc2::NetworkButton wheelRadRevBtn{tuningTable, "WheelRadiusRev"};
  frc2::NetworkButton anglePIDBtn{tuningTable, "AnglePIDTuning"};
  frc2::NetworkButton posePIDBtn{tuningTable, "PosePIDTuning"};


  frc2::CommandPtr RumbleDriver(std::function<units::second_t()> timeToRumble);
  frc2::CommandPtr RumbleOperator(
      std::function<units::second_t()> timeToRumble);

  SwerveSubsystem swerveSubsystem;
  ShooterSubsystem shooterSubsystem;
  IntakeSubsystem intakeSubsystem;
  FeederSubsystem feederSubsystem;
  ClimberSubsystem climberSubsystem;
  str::Vision vision;
  str::NoteVisualizer noteVisualizer;

  Autos autos{swerveSubsystem, shooterSubsystem, intakeSubsystem,
              feederSubsystem};
};
