// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <optional>

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>

#include "RobotContainer.h"
#include "str/SwerveModule.h"
#include "constants/SwerveConstants.h"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void DisabledExit() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void AutonomousExit() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TeleopExit() override;
  void TestInit() override;
  void TestPeriodic() override;
  void TestExit() override;

 private:
  std::optional<frc2::CommandPtr> m_autonomousCommand;

  RobotContainer m_container;

  str::SwerveModule testModule{
    str::SwerveModuleConstants{
      "test",
      consts::swerve::can_ids::FL_STEER,
      consts::swerve::can_ids::FL_DRIVE,
      consts::swerve::can_ids::FL_ENC,
      consts::swerve::physical::FL_ENC_OFFSET,
      consts::swerve::physical::FL_DRIVE_INVERT,
      consts::swerve::physical::FL_STEER_INVERT
    },
    str::SwerveModulePhysical{
      consts::swerve::physical::STEER_GEARING,
      consts::swerve::physical::DRIVE_GEARING,
      consts::swerve::current_limits::SUPPLY_CURRENT_LIMIT,
      consts::swerve::current_limits::SLIP_CURRENT_LIMIT,
      consts::swerve::physical::DRIVE_MOTOR,
      consts::swerve::physical::STEER_MOTOR
    },
    str::SwerveModuleSteerGains{
      consts::swerve::gains::STEER_CRUISE_VEL,
      consts::swerve::gains::STEER_MOTION_MAGIC_KA,
      consts::swerve::gains::STEER_MOTION_MAGIC_KV,
      consts::swerve::gains::STEER_KA,
      consts::swerve::gains::STEER_KV,
      consts::swerve::gains::STEER_KS,
      consts::swerve::gains::STEER_KP,
      consts::swerve::gains::STEER_KI,
      consts::swerve::gains::STEER_KD,
    },
    str::SwerveModuleDriveGains{
      consts::swerve::gains::DRIVE_KA,
      consts::swerve::gains::DRIVE_KV,
      consts::swerve::gains::DRIVE_KS,
      consts::swerve::gains::DRIVE_KP,
      consts::swerve::gains::DRIVE_KI,
      consts::swerve::gains::DRIVE_KD,
    }
  };

  frc::SwerveModuleState desiredState;
};
