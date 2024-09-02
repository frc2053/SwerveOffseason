// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <units/current.h>
#include <units/base.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleTopic.h>
#include <constants/IntakeConstants.h>

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;


 private:
  void UpdateNTEntries();
  bool ConfigureIntakeMotor(bool invert,
                              units::scalar_t intakeGearing,
                              units::ampere_t supplyCurrentLimit,
                              units::ampere_t statorCurrentLimit);
  bool ConfigureMotorSignals();

  std::shared_ptr<nt::NetworkTable> nt{
      nt::NetworkTableInstance::GetDefault().GetTable("Intake")};
};
