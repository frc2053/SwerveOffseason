// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>

#include <functional>
#include <memory>

#include "constants/ClimberConstants.h"

class ClimberSubsystem : public frc2::SubsystemBase {
 public:
  ClimberSubsystem();

  void ConfigureMotor();

  void Periodic() override;
  void SimulationPeriodic() override;
  frc2::CommandPtr ManualControl(std::function<double()> left,
                                 std::function<double()> right);
 private:
  ctre::phoenix6::hardware::TalonFX leftClimberMotor{consts::climber::can_ids::LEFT_MOTOR,
                                                "rio"};
  ctre::phoenix6::hardware::TalonFX rightClimberMotor{consts::climber::can_ids::RIGHT_MOTOR,
                                                "rio"};
};