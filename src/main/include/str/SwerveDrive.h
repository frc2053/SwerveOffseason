// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "str/SwerveModule.h"
#include "constants/SwerveConstants.h"
#include <frc2/command/sysid/SysIdRoutine.h>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <frc/estimator/SwerveDrivePoseEstimator.h>

namespace str {
class SwerveDrive {
public:
  explicit SwerveDrive();
  units::ampere_t GetSimulatedCurrentDraw() const;
  void UpdateSwerveOdom();
  void SimulationPeriodic();
private:
  SwerveModulePhysical swervePhysical{
    consts::swerve::physical::STEER_GEARING,
    consts::swerve::physical::DRIVE_GEARING,
    consts::swerve::current_limits::SUPPLY_CURRENT_LIMIT,
    consts::swerve::current_limits::SLIP_CURRENT_LIMIT,
    consts::swerve::physical::DRIVE_MOTOR,
    consts::swerve::physical::STEER_MOTOR,
    consts::swerve::physical::COUPLING_RATIO,
    consts::swerve::physical::WHEEL_RADIUS,
    consts::swerve::gains::DRIVE_KS_V,
    consts::swerve::gains::STEER_KS_V,
  };

  SwerveModuleSteerGains steerGains{
    consts::swerve::gains::STEER_CRUISE_VEL,
    consts::swerve::gains::STEER_MOTION_MAGIC_KA,
    consts::swerve::gains::STEER_MOTION_MAGIC_KV,
    consts::swerve::gains::STEER_KA,
    consts::swerve::gains::STEER_KV,
    consts::swerve::gains::STEER_KS,
    consts::swerve::gains::STEER_KP,
    consts::swerve::gains::STEER_KI,
    consts::swerve::gains::STEER_KD,
  };

  SwerveModuleDriveGains driveGains{
    consts::swerve::gains::DRIVE_KA,
    consts::swerve::gains::DRIVE_KV,
    consts::swerve::gains::DRIVE_KS,
    consts::swerve::gains::DRIVE_KP,
    consts::swerve::gains::DRIVE_KI,
    consts::swerve::gains::DRIVE_KD,
  };

  std::array<str::SwerveModule, 4> modules{
    SwerveModule{
      str::SwerveModuleConstants{
        "FL",
        consts::swerve::can_ids::FL_STEER,
        consts::swerve::can_ids::FL_DRIVE,
        consts::swerve::can_ids::FL_ENC,
        consts::swerve::physical::FL_ENC_OFFSET,
        consts::swerve::physical::FL_DRIVE_INVERT,
        consts::swerve::physical::FL_STEER_INVERT
      },
      swervePhysical,
      steerGains,
      driveGains
    },
    SwerveModule{
      str::SwerveModuleConstants{
        "FR",
        consts::swerve::can_ids::FR_STEER,
        consts::swerve::can_ids::FR_DRIVE,
        consts::swerve::can_ids::FR_ENC,
        consts::swerve::physical::FR_ENC_OFFSET,
        consts::swerve::physical::FR_DRIVE_INVERT,
        consts::swerve::physical::FR_STEER_INVERT
      },
      swervePhysical,
      steerGains,
      driveGains
    },
    SwerveModule{
      str::SwerveModuleConstants{
        "BL",
        consts::swerve::can_ids::BL_STEER,
        consts::swerve::can_ids::BL_DRIVE,
        consts::swerve::can_ids::BL_ENC,
        consts::swerve::physical::BL_ENC_OFFSET,
        consts::swerve::physical::BL_DRIVE_INVERT,
        consts::swerve::physical::BL_STEER_INVERT
      },
      swervePhysical,
      steerGains,
      driveGains
    },
    SwerveModule{
      str::SwerveModuleConstants{
        "BR",
        consts::swerve::can_ids::BR_STEER,
        consts::swerve::can_ids::BR_DRIVE,
        consts::swerve::can_ids::BR_ENC,
        consts::swerve::physical::BR_ENC_OFFSET,
        consts::swerve::physical::BR_DRIVE_INVERT,
        consts::swerve::physical::BR_STEER_INVERT
      },
      swervePhysical,
      steerGains,
      driveGains
    }
  };

  ctre::phoenix6::hardware::Pigeon2 imu{consts::swerve::can_ids::IMU, "*"};

  std::array<ctre::phoenix6::BaseStatusSignal*, 26> allSignals;

  std::array<frc::SwerveModulePosition, 4> modulePositions;
  std::array<frc::SwerveModuleState, 4> moduleStates;

  frc::SwerveDriveOdometry<4> odom{consts::swerve::physical::KINEMATICS, frc::Rotation2d{0_deg}, modulePositions};
  frc::SwerveDrivePoseEstimator<4> poseEstimator;

  units::second_t lastLoopTime;
};
}