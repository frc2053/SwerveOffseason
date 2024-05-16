// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "str/SwerveModule.h"
#include "constants/SwerveConstants.h"
#include <frc2/command/sysid/SysIdRoutine.h>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/StructTopic.h>
#include <frc/geometry/Pose2d.h>

namespace str {
class SwerveDrive {
public:
  explicit SwerveDrive();
  void ResetPose(const frc::Pose2d& resetPose);
  void DriveRobotRelative(const frc::ChassisSpeeds& robotRelativeSpeeds);
  void Drive(units::meters_per_second_t xVel, units::meters_per_second_t yVel, units::radians_per_second_t omega, bool fieldRelative, bool openLoop=false);
  void SetModuleStates(const std::array<frc::SwerveModuleState, 4>& desiredStates, bool optimize=true, bool openLoop=false);
  void AddVisionMeasurement(const frc::Pose2d& visionMeasurement, units::second_t timestamp, const Eigen::Vector3d& stdDevs);
  units::ampere_t GetSimulatedCurrentDraw() const;
  void UpdateSwerveOdom();
  void UpdateNTEntries();
  frc::ChassisSpeeds GetRobotRelativeSpeeds() const;
  frc::Pose2d GetOdomPose() const;
  frc::Pose2d GetPose() const;
  frc::Pose2d GetPredictedPose(units::second_t translationLookahead, units::second_t rotationLookahead);
  units::radian_t GetYawFromImu();
  std::array<units::radian_t, 4> GetModuleDriveOutputShaftPositions();
  void SimulationPeriodic();
  void SetMk4iCharacterizationTorqueSteer(units::volt_t torqueAmps);
  void SetMk4nCharacterizationTorqueSteer(units::volt_t torqueAmps);
  void SetCharacterizationTorqueDrive(units::volt_t torqueAmps);
  void SetMk4iCharacterizationVoltageSteer(units::volt_t volts);
  void SetMk4nCharacterizationVoltageSteer(units::volt_t volts);
  void SetCharacterizationVoltageDrive(units::volt_t volts);
  void LogMk4iSteerTorque(frc::sysid::SysIdRoutineLog* log);
  void LogMk4nSteerTorque(frc::sysid::SysIdRoutineLog* log);
  void LogDriveTorque(frc::sysid::SysIdRoutineLog* log);
  void LogMk4iSteerVoltage(frc::sysid::SysIdRoutineLog* log);
  void LogMk4nSteerVoltage(frc::sysid::SysIdRoutineLog* log);
  void LogDriveVoltage(frc::sysid::SysIdRoutineLog* log);
private:
  SwerveModulePhysical swervePhysicalFront{
    consts::swerve::physical::STEER_GEARING_MK4I,
    consts::swerve::physical::DRIVE_GEARING,
    consts::swerve::current_limits::SUPPLY_CURRENT_LIMIT,
    consts::swerve::current_limits::STEER_TORQUE_CURRENT_LIMIT,
    consts::swerve::current_limits::SLIP_CURRENT_LIMIT,
    consts::swerve::physical::DRIVE_MOTOR,
    consts::swerve::physical::STEER_MOTOR,
    consts::swerve::physical::COUPLING_RATIO,
    consts::swerve::physical::WHEEL_RADIUS,
    consts::swerve::gains::DRIVE_KS_V,
    consts::swerve::gains::MK4I_STEER_KS_V,
  };

  SwerveModulePhysical swervePhysicalBack{
    consts::swerve::physical::STEER_GEARING_MK4N,
    consts::swerve::physical::DRIVE_GEARING,
    consts::swerve::current_limits::SUPPLY_CURRENT_LIMIT,
    consts::swerve::current_limits::STEER_TORQUE_CURRENT_LIMIT,
    consts::swerve::current_limits::SLIP_CURRENT_LIMIT,
    consts::swerve::physical::DRIVE_MOTOR,
    consts::swerve::physical::STEER_MOTOR,
    consts::swerve::physical::COUPLING_RATIO,
    consts::swerve::physical::WHEEL_RADIUS,
    consts::swerve::gains::DRIVE_KS_V,
    consts::swerve::gains::MK4N_STEER_KS_V,
  };

  SwerveModuleSteerGains steerGainsMk4i{
    consts::swerve::gains::MK4I_STEER_CRUISE_VEL,
    consts::swerve::gains::MK4I_STEER_MOTION_MAGIC_KA,
    consts::swerve::gains::MK4I_STEER_MOTION_MAGIC_KV,
    consts::swerve::gains::MK4I_STEER_KA,
    consts::swerve::gains::MK4I_STEER_KV,
    consts::swerve::gains::MK4I_STEER_KS,
    consts::swerve::gains::MK4I_STEER_KP,
    consts::swerve::gains::MK4I_STEER_KI,
    consts::swerve::gains::MK4I_STEER_KD,
  };

  SwerveModuleSteerGains steerGainsMk4n{
    consts::swerve::gains::MK4N_STEER_CRUISE_VEL,
    consts::swerve::gains::MK4N_STEER_MOTION_MAGIC_KA,
    consts::swerve::gains::MK4N_STEER_MOTION_MAGIC_KV,
    consts::swerve::gains::MK4N_STEER_KA,
    consts::swerve::gains::MK4N_STEER_KV,
    consts::swerve::gains::MK4N_STEER_KS,
    consts::swerve::gains::MK4N_STEER_KP,
    consts::swerve::gains::MK4N_STEER_KI,
    consts::swerve::gains::MK4N_STEER_KD,
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
      swervePhysicalFront,
      steerGainsMk4i,
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
      swervePhysicalFront,
      steerGainsMk4i,
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
      swervePhysicalBack,
      steerGainsMk4n,
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
      swervePhysicalBack,
      steerGainsMk4n,
      driveGains
    }
  };

  ctre::phoenix6::hardware::Pigeon2 imu{consts::swerve::can_ids::IMU, "*"};
  ctre::phoenix6::sim::Pigeon2SimState& imuSimState = imu.GetSimState();

  std::array<ctre::phoenix6::BaseStatusSignal*, 34> allSignals;

  std::array<frc::SwerveModulePosition, 4> modulePositions;
  std::array<frc::SwerveModuleState, 4> moduleStates;

  frc::SwerveDriveOdometry<4> odom{consts::swerve::physical::KINEMATICS, frc::Rotation2d{0_deg}, modulePositions};
  frc::SwerveDrivePoseEstimator<4> poseEstimator{consts::swerve::physical::KINEMATICS, frc::Rotation2d{0_deg}, modulePositions, frc::Pose2d{}};

  units::radian_t yawLatencyComped;
  units::second_t lastDriveLoopTime;
  units::second_t lastSimLoopTime;
  frc::Rotation2d lastSimAngle;

  std::shared_ptr<nt::NetworkTable> nt{nt::NetworkTableInstance::GetDefault().GetTable("SwerveDrive")};
  nt::DoublePublisher loopTimePub{nt::NetworkTableInstance::GetDefault().GetTable("Metadata")->GetDoubleTopic("SwerveDriveOdomLoopRate").Publish()};
  nt::StructArrayPublisher<frc::SwerveModuleState> desiredStatesPub{nt->GetStructArrayTopic<frc::SwerveModuleState>("DesiredStates").Publish()};
  nt::StructArrayPublisher<frc::SwerveModuleState> currentStatesPub{nt->GetStructArrayTopic<frc::SwerveModuleState>("CurrentStates").Publish()};
  nt::StructArrayPublisher<frc::SwerveModuleState> simStatesPub{nt->GetStructArrayTopic<frc::SwerveModuleState>("SimStates").Publish()};
  nt::StructArrayPublisher<frc::SwerveModulePosition> currentPositionsPub{nt->GetStructArrayTopic<frc::SwerveModulePosition>("CurrentPositions").Publish()};
  nt::StructPublisher<frc::Pose2d> odomPosePub{nt->GetStructTopic<frc::Pose2d>("OdometryPose").Publish()};
  nt::StructPublisher<frc::Pose2d> estimatorPub{nt->GetStructTopic<frc::Pose2d>("PoseEstimatorPose").Publish()};
  nt::StructPublisher<frc::Pose2d> lookaheadPub{nt->GetStructTopic<frc::Pose2d>("LookaheadPose").Publish()};
};
}