// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <str/SwerveDrive.h>

class SwerveSubsystem : public frc2::SubsystemBase {
 public:
  SwerveSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SimulationPeriodic() override;
  units::ampere_t GetSimulatedCurrentDraw() const;
  frc::Pose2d GetOdomPose();
  frc::Pose2d GetRobotPose();
  void UpdateSwerveOdom();
  frc2::CommandPtr PointWheelsToAngle(std::function<units::radian_t()> wheelAngle);
  frc2::CommandPtr XPattern();
  frc2::CommandPtr Drive(std::function<units::meters_per_second_t()> xVel, std::function<units::meters_per_second_t()> yVel, std::function<units::radians_per_second_t()> omega, bool fieldRelative);
  frc2::CommandPtr AlignToAmp(); 
  frc2::CommandPtr SysIdSteerQuasistaticTorque(frc2::sysid::Direction dir);
  frc2::CommandPtr SysIdSteerDynamicTorque(frc2::sysid::Direction dir);
  frc2::CommandPtr SysIdDriveQuasistaticTorque(frc2::sysid::Direction dir);
  frc2::CommandPtr SysIdDriveDynamicTorque(frc2::sysid::Direction dir);
  frc2::CommandPtr SysIdSteerQuasistaticVoltage(frc2::sysid::Direction dir);
  frc2::CommandPtr SysIdSteerDynamicVoltage(frc2::sysid::Direction dir);
  frc2::CommandPtr SysIdDriveQuasistaticVoltage(frc2::sysid::Direction dir);
  frc2::CommandPtr SysIdDriveDynamicVoltage(frc2::sysid::Direction dir);
  frc2::CommandPtr WheelRadius(frc2::sysid::Direction dir);
 private:
  void SetupPathplanner();

  str::SwerveDrive swerveDrive;
  str::WheelRadiusCharData wheelRadData;

  //It says volts, because sysid only supports volts for now. But we are using current anyway
  frc2::sysid::SysIdRoutine steerTorqueSysid{
    frc2::sysid::Config{
      10_V / 1_s,
      30_V,
      10_s,
      [this](frc::sysid::State state) {
        ctre::phoenix6::SignalLogger().WriteString(
            "state", frc::sysid::SysIdRoutineLog::StateEnumToString(state));
      }
    },
    frc2::sysid::Mechanism{
      [this](units::volt_t voltsToSend) {
        swerveDrive.SetCharacterizationTorqueSteer(voltsToSend);
      },
      [this](frc::sysid::SysIdRoutineLog* log) {
        swerveDrive.LogSteerTorque(log);
      },
      this,
      "swerve-steer"
    }
  };

  frc2::sysid::SysIdRoutine steerVoltageSysid{
    frc2::sysid::Config{
      std::nullopt,
      std::nullopt,
      std::nullopt,
      [this](frc::sysid::State state) {
        ctre::phoenix6::SignalLogger().WriteString(
            "state", frc::sysid::SysIdRoutineLog::StateEnumToString(state));
      }
    },
    frc2::sysid::Mechanism{
      [this](units::volt_t voltsToSend) {
        swerveDrive.SetCharacterizationVoltageSteer(voltsToSend);
      },
      [this](frc::sysid::SysIdRoutineLog* log) {
        swerveDrive.LogSteerVoltage(log);
      },
      this,
      "swerve-steer"
    }
  };

  //It says volts, because sysid only supports volts for now. But we are using current anyway
  frc2::sysid::SysIdRoutine driveTorqueSysid{
    frc2::sysid::Config{
      10_V / 1_s,
      30_V,
      10_s,
      [this](frc::sysid::State state) {
        ctre::phoenix6::SignalLogger().WriteString(
            "state", frc::sysid::SysIdRoutineLog::StateEnumToString(state));
      }
    },
    frc2::sysid::Mechanism{
      [this](units::volt_t voltsToSend) {
        swerveDrive.SetCharacterizationTorqueDrive(voltsToSend);
      },
      [this](frc::sysid::SysIdRoutineLog* log) {
        swerveDrive.LogDriveTorque(log);
      },
      this,
      "swerve-drive"
    }
  };

  frc2::sysid::SysIdRoutine driveVoltageSysid{
    frc2::sysid::Config{
      std::nullopt,
      std::nullopt,
      std::nullopt,
      [this](frc::sysid::State state) {
        ctre::phoenix6::SignalLogger().WriteString(
            "state", frc::sysid::SysIdRoutineLog::StateEnumToString(state));
      }
    },
    frc2::sysid::Mechanism{
      [this](units::volt_t voltsToSend) {
        swerveDrive.SetCharacterizationVoltageDrive(voltsToSend);
      },
      [this](frc::sysid::SysIdRoutineLog* log) {
        swerveDrive.LogDriveVoltage(log);
      },
      this,
      "swerve-drive"
    }
  };
};
