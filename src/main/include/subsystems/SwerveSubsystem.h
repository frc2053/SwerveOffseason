// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <choreo/auto/AutoFactory.h>
#include <choreo/trajectory/SwerveSample.h>
#include <choreo/trajectory/Trajectory.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/SubsystemBase.h>
#include <networktables/StructArrayTopic.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <str/DriverstationUtils.h>
#include <str/SwerveDrive.h>

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include "constants/Constants.h"

enum class SpeakerSide {
    AMP_SIDE,
    CENTER,
    SOURCE
};

class SwerveSubsystem : public frc2::SubsystemBase {
 public:
  SwerveSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SimulationPeriodic() override;
  frc2::CommandPtr ZeroYaw();
  units::ampere_t GetSimulatedCurrentDraw() const;
  frc::Pose2d GetOdomPose();
  frc::Pose2d GetRobotPose();
  units::meter_t GetDistanceToSpeaker(const SpeakerSide& side);
  frc::ChassisSpeeds GetFieldRelativeSpeed();
  frc::ChassisSpeeds GetRobotRelativeSpeed() const;
  void ResetPose(frc::Pose2d resetPose) { swerveDrive.ResetPose(resetPose); }
  void UpdateSwerveOdom();
  void AddVisionMeasurement(const frc::Pose2d& visionMeasurement,
                            units::second_t timestamp,
                            const Eigen::Vector3d& stdDevs);
  frc2::CommandPtr PointWheelsToAngle(
      std::function<units::radian_t()> wheelAngle);
  frc2::CommandPtr XPattern();
  frc2::CommandPtr Drive(std::function<units::meters_per_second_t()> xVel,
                         std::function<units::meters_per_second_t()> yVel,
                         std::function<units::radians_per_second_t()> omega,
                         bool fieldRelative, bool openLoop = false);
  frc2::CommandPtr PIDToPose(std::function<frc::Pose2d()> goalPose);
  frc2::CommandPtr AlignToAmp();
  SpeakerSide GetSpeakerSideFromPosition();
  frc2::CommandPtr RotateToAngle(std::function<units::meters_per_second_t()> xVel, std::function<units::meters_per_second_t()> yVel, std::function<units::radian_t()> goalAngle, std::function<bool()> override);
  frc2::CommandPtr FaceSpeaker(std::function<units::meters_per_second_t()> xVel, std::function<units::meters_per_second_t()> yVel);
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
  frc2::CommandPtr TuneDrivePID(std::function<bool()> isDone);
  void CalculateFoundNotePose(std::optional<units::meter_t> distanceToNote,
                              std::optional<units::radian_t> angleToNote);
  frc::Pose2d GetFoundNotePose() const;
  frc2::CommandPtr NoteAssist(
      std::function<units::meters_per_second_t()> xVel,
      std::function<units::meters_per_second_t()> yVel,
      std::function<units::radians_per_second_t()> rotOverride,
      std::function<frc::Pose2d()> notePose);
  choreo::AutoFactory<choreo::SwerveSample>& GetFactory() {
    return autoFactory;
  }

  frc2::CommandPtr Stop();

 private:
  void SetupPathplanner();
  frc::Translation2d GetAmpLocation();
  frc::Translation2d GetFrontAmpLocation();
  bool IsNearAmp();

  units::meter_t cachedNoteDist;
  frc::Pose2d latestNotePose;

  std::shared_ptr<pathplanner::PPHolonomicDriveController> ppControllers;

  nt::StructPublisher<frc::Pose3d> foundNotePose{
      nt::NetworkTableInstance::GetDefault()
          .GetTable("Vision")
          ->GetStructTopic<frc::Pose3d>("FoundNotePose")
          .Publish()};
  nt::DoublePublisher noteDistPub{nt::NetworkTableInstance::GetDefault()
                                      .GetTable("Vision")
                                      ->GetDoubleTopic("NoteDist")
                                      .Publish()};
  nt::DoublePublisher noteYawPub{nt::NetworkTableInstance::GetDefault()
                                     .GetTable("Vision")
                                     ->GetDoubleTopic("NoteYaw")
                                     .Publish()};

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

  nt::StructArrayPublisher<frc::Pose2d> choreoTrajectoryPub{
      nt->GetStructArrayTopic<frc::Pose2d>("CurrentChoreoTrajectory")
          .Publish()};

  frc::PIDController xController{consts::swerve::pathplanning::POSE_P,
                                 consts::swerve::pathplanning::POSE_I,
                                 consts::swerve::pathplanning::POSE_D};
  frc::PIDController yController{consts::swerve::pathplanning::POSE_P,
                                 consts::swerve::pathplanning::POSE_I,
                                 consts::swerve::pathplanning::POSE_D};
  frc::PIDController rotationController{
      consts::swerve::pathplanning::ROTATION_P,
      consts::swerve::pathplanning::ROTATION_I,
      consts::swerve::pathplanning::ROTATION_D};

  choreo::AutoFactory<choreo::SwerveSample> autoFactory{
      [this] { return GetRobotPose(); },
      [this](frc::Pose2d refPose, choreo::SwerveSample currentSample) {
        units::meters_per_second_t xFF = currentSample.vx;
        units::meters_per_second_t yFF = currentSample.vy;
        units::radians_per_second_t rotationFF = currentSample.omega;

        units::meters_per_second_t xFeedback{xController.Calculate(
            refPose.X().value(), currentSample.x.value())};
        units::meters_per_second_t yFeedback{yController.Calculate(
            refPose.Y().value(), currentSample.y.value())};
        units::radians_per_second_t rotationFeedback{
            rotationController.Calculate(refPose.Rotation().Radians().value(),
                                         currentSample.heading.value())};

        auto speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
            xFF + xFeedback, yFF + yFeedback, rotationFF + rotationFeedback,
            refPose.Rotation());
        swerveDrive.SetXModuleForces(currentSample.moduleForcesX);
        swerveDrive.SetYModuleForces(currentSample.moduleForcesY);
        swerveDrive.DriveRobotRelative(speeds);
      },
      [] { return str::IsOnRed(); },
      {this},
      [this](choreo::Trajectory<choreo::SwerveSample> trajectory,
             bool starting) {
        if (!starting) {
          choreoTrajectoryPub.Set({});
        } else {
          choreoTrajectoryPub.Set(trajectory.GetPoses());
        }
      }};

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
                             [this](frc::sysid::SysIdRoutineLog* log) {
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
                             [this](frc::sysid::SysIdRoutineLog* log) {
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
                             [this](frc::sysid::SysIdRoutineLog* log) {
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
                             [this](frc::sysid::SysIdRoutineLog* log) {
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
                             [this](frc::sysid::SysIdRoutineLog* log) {
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
                             [this](frc::sysid::SysIdRoutineLog* log) {
                               swerveDrive.LogDriveVoltage(log);
                             },
                             this, "swerve-drive"}};
};
