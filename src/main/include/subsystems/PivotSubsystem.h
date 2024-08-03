// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

#include "constants/PivotConstants.h"

class PivotSubsystem : public frc2::SubsystemBase {
public:
  PivotSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void GoToAngle(units::radian_t targetAngle);

  units::radian_t GetAngle();
  units::radian_t GetTargetAngle();

  bool AtTargetAngle();

private:
  bool ConfigurePivotMotors();
  bool ConfigurePivotEncoder();
  void ConfigureControlSignals();

  ctre::phoenix6::hardware::TalonFX pivotLeft{
      consts::pivot::can_ids::LEFT_PIVOT};
  ctre::phoenix6::hardware::TalonFX pivotRight{
      consts::pivot::can_ids::RIGHT_PIVOT};
  ctre::phoenix6::hardware::CANcoder pivotEncoder{
      consts::pivot::can_ids::PIVOT_ENCODER};

  ctre::phoenix6::StatusSignal<units::turn_t> pivotPositionSignal =
      pivotLeft.GetPosition();
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> pivotVelocitySignal =
      pivotLeft.GetVelocity();

  ctre::phoenix6::controls::MotionMagicTorqueCurrentFOC pivotAngleSetter{0_rad};

  units::radian_t currentPivotAngle = 0_rad;
  units::radian_t targetedPivotAngle = 0_rad;
};
