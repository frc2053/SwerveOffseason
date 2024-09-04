// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <choreo/lib/Choreo.h>
#include <choreo/lib/ChoreoSwerveCommand.h>
#include <choreo/lib/ChoreoTrajectory.h>
#include <frc2/command/SubsystemBase.h>
#include <networktables/StructArrayTopic.h>
#include <str/SwerveDrive.h>

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include "constants/Constants.h"

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
  frc::ChassisSpeeds GetFieldRelativeSpeed();
  frc::ChassisSpeeds GetRobotRelativeSpeed() const;
  void UpdateSwerveOdom();
  void AddVisionMeasurement(const frc::Pose2d &visionMeasurement,
                            units::second_t timestamp,
                            const Eigen::Vector3d &stdDevs);
  frc2::CommandPtr
  FollowChoreoTrajectory(std::function<std::string()> pathName);
  frc2::CommandPtr
  PointWheelsToAngle(std::function<units::radian_t()> wheelAngle);
  frc2::CommandPtr XPattern();
  frc2::CommandPtr Drive(std::function<units::meters_per_second_t()> xVel,
                         std::function<units::meters_per_second_t()> yVel,
                         std::function<units::radians_per_second_t()> omega,
                         bool fieldRelative, bool openLoop = false);
  frc2::CommandPtr PIDToPose(std::function<frc::Pose2d()> goalPose);
  frc2::CommandPtr AlignToAmp();
  frc2::CommandPtr SysIdSteerMk4iQuasistaticTorque(frc2::sysid::Direction dir);
  frc2::CommandPtr SysIdSteerMk4iDynamicTorque(frc2::sysid::Direction dir);
  frc2::CommandPtr SysIdSteerMk4nQuasistaticTorque(frc2::sysid::Direction dir);
  frc2::CommandPtr SysIdSteerMk4nDynamicTorque(frc2::sysid::Direction dir);
  frc2::CommandPtr SysIdDriveQuasistaticTorque(frc2::sysid::Direction dir);
  frc2::CommandPtr SysIdDriveDynamicTorque(frc2::sysid::Direction dir);
  frc2::CommandPtr SysIdSteerMk4iQuasistaticVoltage(frc2::sysid::Direction dir);
  frc2::CommandPtr SysIdSteerMk4iDynamicVoltage(frc2::sysid::Direction dir);
  frc2::CommandPtr SysIdSteerMk4nQuasistaticVoltage(frc2::sysid::Direction dir);
  frc2::CommandPtr SysIdSteerMk4nDynamicVoltage(frc2::sysid::Direction dir);
  frc2::CommandPtr SysIdDriveQuasistaticVoltage(frc2::sysid::Direction dir);
  frc2::CommandPtr SysIdDriveDynamicVoltage(frc2::sysid::Direction dir);
  frc2::CommandPtr WheelRadius(frc2::sysid::Direction dir);
  frc2::CommandPtr TuneSteerPID(std::function<bool()> isDone);


private:
  void SetupPathplanner();
  void LoadChoreoTrajectories();
  frc::Translation2d GetAmpLocation();
  frc::Translation2d GetFrontAmpLocation();
  bool IsNearAmp();

  frc::TrapezoidProfile<units::meters>::Constraints translationConstraints{
      consts::swerve::physical::DRIVE_MAX_SPEED,
      consts::swerve::physical::DRIVE_MAX_ACCEL,
  };

  frc::TrapezoidProfile<units::radians>::Constraints rotationConstraints{
      consts::swerve::physical::DRIVE_MAX_ROT_SPEED,
      consts::swerve::physical::DRIVE_MAX_ROT_ACCEL,
  };

  frc::ProfiledPIDController<units::meters> xPoseController{
      consts::swerve::pathplanning::POSE_P,
      consts::swerve::pathplanning::POSE_I,
      consts::swerve::pathplanning::POSE_D, translationConstraints,
      consts::LOOP_PERIOD};

  frc::ProfiledPIDController<units::meters> yPoseController{
      consts::swerve::pathplanning::POSE_P,
      consts::swerve::pathplanning::POSE_I,
      consts::swerve::pathplanning::POSE_D, translationConstraints,
      consts::LOOP_PERIOD};

  frc::ProfiledPIDController<units::radians> thetaController{
      consts::swerve::pathplanning::ROTATION_P,
      consts::swerve::pathplanning::ROTATION_I,
      consts::swerve::pathplanning::ROTATION_D, rotationConstraints,
      consts::LOOP_PERIOD};

  str::SwerveDrive swerveDrive;
  str::WheelRadiusCharData wheelRadData;

  std::shared_ptr<nt::NetworkTable> nt{
      nt::NetworkTableInstance::GetDefault().GetTable("SwerveDrive")};
  nt::StructPublisher<frc::Pose2d> pidPoseSetpointPub{
      nt->GetStructTopic<frc::Pose2d>("PIDToPoseSetpoint").Publish()};

  choreolib::ChoreoControllerFunction choreoController;
  std::unordered_map<std::string, choreolib::ChoreoTrajectory> pathMap;
  nt::StructArrayPublisher<frc::Pose2d> choreoTrajectoryPub{
      nt->GetStructArrayTopic<frc::Pose2d>("CurrentChoreoTrajectory")
          .Publish()};

  // It says volts, because sysid only supports volts for now. But we are using
  // current anyway
  frc2::sysid::SysIdRoutine steerMk4iTorqueSysid{
      frc2::sysid::Config{
          10_V / 1_s, 30_V, 10_s,
          [this](frc::sysid::State state) {
            ctre::phoenix6::SignalLogger().WriteString(
                "state", frc::sysid::SysIdRoutineLog::StateEnumToString(state));
          }},
      frc2::sysid::Mechanism{[this](units::volt_t voltsToSend) {
                               swerveDrive.SetMk4iCharacterizationTorqueSteer(
                                   voltsToSend);
                             },
                             [this](frc::sysid::SysIdRoutineLog *log) {
                               swerveDrive.LogMk4iSteerTorque(log);
                             },
                             this, "swerve-steer-mk4i"}};

  // It says volts, because sysid only supports volts for now. But we are using
  // current anyway
  frc2::sysid::SysIdRoutine steerMk4nTorqueSysid{
      frc2::sysid::Config{
          10_V / 1_s, 30_V, 10_s,
          [this](frc::sysid::State state) {
            ctre::phoenix6::SignalLogger().WriteString(
                "state", frc::sysid::SysIdRoutineLog::StateEnumToString(state));
          }},
      frc2::sysid::Mechanism{[this](units::volt_t voltsToSend) {
                               swerveDrive.SetMk4nCharacterizationTorqueSteer(
                                   voltsToSend);
                             },
                             [this](frc::sysid::SysIdRoutineLog *log) {
                               swerveDrive.LogMk4nSteerTorque(log);
                             },
                             this, "swerve-steer-mk4n"}};

  frc2::sysid::SysIdRoutine steerMk4iVoltageSysid{
      frc2::sysid::Config{
          std::nullopt, std::nullopt, std::nullopt,
          [this](frc::sysid::State state) {
            ctre::phoenix6::SignalLogger().WriteString(
                "state", frc::sysid::SysIdRoutineLog::StateEnumToString(state));
          }},
      frc2::sysid::Mechanism{[this](units::volt_t voltsToSend) {
                               swerveDrive.SetMk4iCharacterizationVoltageSteer(
                                   voltsToSend);
                             },
                             [this](frc::sysid::SysIdRoutineLog *log) {
                               swerveDrive.LogMk4iSteerVoltage(log);
                             },
                             this, "swerve-steer-mk4i"}};

  frc2::sysid::SysIdRoutine steerMk4nVoltageSysid{
      frc2::sysid::Config{
          std::nullopt, std::nullopt, std::nullopt,
          [this](frc::sysid::State state) {
            ctre::phoenix6::SignalLogger().WriteString(
                "state", frc::sysid::SysIdRoutineLog::StateEnumToString(state));
          }},
      frc2::sysid::Mechanism{[this](units::volt_t voltsToSend) {
                               swerveDrive.SetMk4nCharacterizationVoltageSteer(
                                   voltsToSend);
                             },
                             [this](frc::sysid::SysIdRoutineLog *log) {
                               swerveDrive.LogMk4nSteerVoltage(log);
                             },
                             this, "swerve-steer-mk4n"}};

  // It says volts, because sysid only supports volts for now. But we are using
  // current anyway
  frc2::sysid::SysIdRoutine driveTorqueSysid{
      frc2::sysid::Config{
          10_V / 1_s, 30_V, 10_s,
          [this](frc::sysid::State state) {
            ctre::phoenix6::SignalLogger().WriteString(
                "state", frc::sysid::SysIdRoutineLog::StateEnumToString(state));
          }},
      frc2::sysid::Mechanism{[this](units::volt_t voltsToSend) {
                               swerveDrive.SetCharacterizationTorqueDrive(
                                   voltsToSend);
                             },
                             [this](frc::sysid::SysIdRoutineLog *log) {
                               swerveDrive.LogDriveTorque(log);
                             },
                             this, "swerve-drive"}};

  frc2::sysid::SysIdRoutine driveVoltageSysid{
      frc2::sysid::Config{
          std::nullopt, std::nullopt, std::nullopt,
          [this](frc::sysid::State state) {
            ctre::phoenix6::SignalLogger().WriteString(
                "state", frc::sysid::SysIdRoutineLog::StateEnumToString(state));
          }},
      frc2::sysid::Mechanism{[this](units::volt_t voltsToSend) {
                               swerveDrive.SetCharacterizationVoltageDrive(
                                   voltsToSend);
                             },
                             [this](frc::sysid::SysIdRoutineLog *log) {
                               swerveDrive.LogDriveVoltage(log);
                             },
                             this, "swerve-drive"}};
};
