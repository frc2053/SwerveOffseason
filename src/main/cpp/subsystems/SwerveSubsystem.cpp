// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "subsystems/SwerveSubsystem.h"

#include <choreo/lib/Choreo.h>
#include <frc/Filesystem.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <str/ChoreoSwerveCommandWithForce.h>
#include <str/DriverstationUtils.h>

#include "constants/Constants.h"

SwerveSubsystem::SwerveSubsystem()
    : choreoController(choreolib::Choreo::ChoreoSwerveController(
          frc::PIDController{consts::swerve::pathplanning::POSE_P,
                             consts::swerve::pathplanning::POSE_I,
                             consts::swerve::pathplanning::POSE_D},
          frc::PIDController{consts::swerve::pathplanning::POSE_P,
                             consts::swerve::pathplanning::POSE_I,
                             consts::swerve::pathplanning::POSE_D},
          frc::PIDController{consts::swerve::pathplanning::ROTATION_P,
                             consts::swerve::pathplanning::ROTATION_I,
                             consts::swerve::pathplanning::ROTATION_D})) {
  SetName("SwerveSubsystem");
  frc::SmartDashboard::PutData(this);
  SetupPathplanner();
  LoadChoreoTrajectories();
}

void SwerveSubsystem::UpdateSwerveOdom() { swerveDrive.UpdateSwerveOdom(); }

void SwerveSubsystem::AddVisionMeasurement(const frc::Pose2d &visionMeasurement,
                                           units::second_t timestamp,
                                           const Eigen::Vector3d &stdDevs) {
  swerveDrive.AddVisionMeasurement(visionMeasurement, timestamp, stdDevs);
}

// This method will be called once per scheduler run
void SwerveSubsystem::Periodic() { swerveDrive.UpdateNTEntries(); }

void SwerveSubsystem::SimulationPeriodic() { swerveDrive.SimulationPeriodic(); }

frc::Pose2d SwerveSubsystem::GetOdomPose() { return swerveDrive.GetOdomPose(); }

frc::Pose2d SwerveSubsystem::GetRobotPose() { return swerveDrive.GetPose(); }

frc::ChassisSpeeds SwerveSubsystem::GetFieldRelativeSpeed() {
  return frc::ChassisSpeeds::FromRobotRelativeSpeeds(
      swerveDrive.GetRobotRelativeSpeeds(), GetRobotPose().Rotation());
}

frc::ChassisSpeeds SwerveSubsystem::GetRobotRelativeSpeed() const {
  return swerveDrive.GetRobotRelativeSpeeds();
}

units::ampere_t SwerveSubsystem::GetSimulatedCurrentDraw() const {
  return swerveDrive.GetSimulatedCurrentDraw();
}

frc2::CommandPtr
SwerveSubsystem::FollowChoreoTrajectory(std::function<std::string()> pathName) {
  return frc2::cmd::Either(
      frc2::cmd::Sequence(
          frc2::cmd::RunOnce([this, pathName] {
            if (str::IsOnRed()) {
              choreolib::ChoreoTrajectory flippedTraj =
                  pathMap[pathName()].Flipped();
              swerveDrive.ResetPose(flippedTraj.GetInitialPose());
              choreoTrajectoryPub.Set(flippedTraj.GetPoses());
            } else {
              swerveDrive.ResetPose(pathMap[pathName()].GetInitialPose());
              choreoTrajectoryPub.Set(pathMap[pathName()].GetPoses());
            }
          }),
          str::ChoreoSwerveCommandWithForce(
              pathMap[pathName()], [this] { return swerveDrive.GetPose(); },
              choreoController,
              [this](frc::ChassisSpeeds speeds) {
                swerveDrive.DriveRobotRelative(speeds);
              },
              [this](std::array<units::newton_t, 4> xForce) {
                swerveDrive.SetXModuleForces(xForce);
              },
              [this](std::array<units::newton_t, 4> yForce) {
                swerveDrive.SetYModuleForces(yForce);
              },
              [this] { return str::IsOnRed(); }, {this})
              .ToPtr(),
          frc2::cmd::RunOnce(
              [this] { swerveDrive.Drive(0_mps, 0_mps, 0_rad_per_s, false); })),
      frc2::cmd::Print(
          "ERROR: Choreo path wasn't found in pathMap!!!!\n\n\n\n"),
      [this, pathName] { return pathMap.contains(pathName()); });
}

frc2::CommandPtr SwerveSubsystem::PointWheelsToAngle(
    std::function<units::radian_t()> wheelAngle) {
  return frc2::cmd::RunOnce(
             [this, wheelAngle] {
               std::array<frc::SwerveModuleState, 4> states;
               for (auto &state : states) {
                 state.angle = wheelAngle();
                 state.speed = 0_mps;
               }

               swerveDrive.SetModuleStates(states, false);
             },
             {this})
      .WithName("Point Wheels At Angle");
}

frc2::CommandPtr SwerveSubsystem::XPattern() {
  return frc2::cmd::Run(
             [this] {
               std::array<frc::SwerveModuleState, 4> states;

               states[0].angle = 45_deg;
               states[1].angle = -45_deg;
               states[2].angle = -45_deg;
               states[3].angle = 45_deg;

               swerveDrive.SetModuleStates(states, true);
             },
             {this})
      .WithName("X Pattern");
}

void SwerveSubsystem::SetupPathplanner() {
  pathplanner::AutoBuilder::configureHolonomic(
      [this] { return swerveDrive.GetPose(); },
      [this](frc::Pose2d resetPose) {
        return swerveDrive.ResetPose(resetPose);
      },
      [this] { return swerveDrive.GetRobotRelativeSpeeds(); },
      [this](frc::ChassisSpeeds robotRelativeOutput) {
        swerveDrive.DriveRobotRelative(robotRelativeOutput);
      },
      consts::swerve::pathplanning::PATH_CONFIG, [] { return str::IsOnRed(); },
      this);
}

void SwerveSubsystem::LoadChoreoTrajectories() {
  for (const auto &entry : std::filesystem::directory_iterator(
           frc::filesystem::GetDeployDirectory() + "/choreo/")) {
    std::string fileName = entry.path().stem().string();
    fmt::print("Loaded choreo trajectory: {}\n", fileName);
    pathMap[fileName] = choreolib::Choreo::GetTrajectory(fileName);
  }
}

frc::Translation2d SwerveSubsystem::GetAmpLocation() {
  frc::Translation2d ampToGoTo = consts::yearSpecific::ampLocation;
  if (str::IsOnRed()) {
    ampToGoTo = pathplanner::GeometryUtil::flipFieldPosition(ampToGoTo);
  }
  return ampToGoTo;
}

frc::Translation2d SwerveSubsystem::GetFrontAmpLocation() {
  frc::Translation2d ampToGoTo = consts::yearSpecific::inFrontOfAmpLocation;
  if (str::IsOnRed()) {
    ampToGoTo = pathplanner::GeometryUtil::flipFieldPosition(ampToGoTo);
  }
  return ampToGoTo;
}

bool SwerveSubsystem::IsNearAmp() {
  return GetRobotPose().Translation().Distance(GetAmpLocation()) <
         consts::yearSpecific::closeToAmpDistance;
}

frc2::CommandPtr
SwerveSubsystem::Drive(std::function<units::meters_per_second_t()> xVel,
                       std::function<units::meters_per_second_t()> yVel,
                       std::function<units::radians_per_second_t()> omega,
                       bool fieldRelative, bool openLoop) {
  return frc2::cmd::Run(
             [this, xVel, yVel, omega, fieldRelative, openLoop] {
               swerveDrive.Drive(xVel(), yVel(), omega(), fieldRelative,
                                 openLoop);
             },
             {this})
      .WithName("Drive Command");
}

frc2::CommandPtr
SwerveSubsystem::PIDToPose(std::function<frc::Pose2d()> goalPose) {
  return frc2::cmd::Sequence(
             frc2::cmd::RunOnce(
                 [this, goalPose] {
                   frc::Pose2d currentPose = GetRobotPose();
                   frc::ChassisSpeeds currentSpeeds = GetFieldRelativeSpeed();
                   xPoseController.Reset(currentPose.Translation().X(),
                                         currentSpeeds.vx);
                   yPoseController.Reset(currentPose.Translation().Y(),
                                         currentSpeeds.vy);
                   thetaController.Reset(currentPose.Rotation().Radians(),
                                         currentSpeeds.omega);
                   thetaController.EnableContinuousInput(
                       units::radian_t{-std::numbers::pi},
                       units::radian_t{std::numbers::pi});
                   xPoseController.SetGoal(goalPose().X());
                   yPoseController.SetGoal(goalPose().Y());
                   thetaController.SetGoal(goalPose().Rotation().Radians());
                   xPoseController.SetTolerance(
                       consts::swerve::pathplanning::translationalPIDTolerance,
                       consts::swerve::pathplanning::
                           translationalVelPIDTolerance);
                   yPoseController.SetTolerance(
                       consts::swerve::pathplanning::translationalPIDTolerance,
                       consts::swerve::pathplanning::
                           translationalVelPIDTolerance);
                   thetaController.SetTolerance(
                       consts::swerve::pathplanning::rotationalPIDTolerance,
                       consts::swerve::pathplanning::rotationalVelPIDTolerance);
                   pidPoseSetpointPub.Set(goalPose());
                 },
                 {this})
                 .WithName("PIDToPose Init"),
             frc2::cmd::Run(
                 [this, goalPose] {
                   frc::Pose2d currentPose = GetRobotPose();

                   units::meters_per_second_t xSpeed{xPoseController.Calculate(
                       currentPose.Translation().X())};
                   units::meters_per_second_t ySpeed{yPoseController.Calculate(
                       currentPose.Translation().Y())};
                   units::radians_per_second_t thetaSpeed{
                       thetaController.Calculate(
                           currentPose.Rotation().Radians())};

                   swerveDrive.Drive(xSpeed, ySpeed, thetaSpeed, true);
                 },
                 {this})
                 .Until([this] {
                   return xPoseController.AtGoal() &&
                          yPoseController.AtGoal() && thetaController.AtGoal();
                 })
                 .WithName("PIDToPose Run"),
             frc2::cmd::Run([this] {
               swerveDrive.Drive(0_mps, 0_mps, 0_deg_per_s, false);
             }).WithName("PIDToPose Stop"))
      .WithName("PIDToPose");
}

frc2::CommandPtr SwerveSubsystem::AlignToAmp() {
  // If we are close enough to the amp, just pid there
  return frc2::cmd::Either(
             PIDToPose([this] {
               return frc::Pose2d{GetAmpLocation(), frc::Rotation2d{90_deg}};
             }),
             frc2::cmd::Sequence(
                 pathplanner::AutoBuilder::pathfindToPoseFlipped(
                     frc::Pose2d{GetFrontAmpLocation(),
                                 frc::Rotation2d{90_deg}},
                     consts::swerve::pathplanning::constraints, 3_fps),
                 PIDToPose([this] {
                   return frc::Pose2d{GetAmpLocation(),
                                      frc::Rotation2d{90_deg}};
                 })),
             [this] { return IsNearAmp(); })
      .WithName("AlignToAmp");
}

frc2::CommandPtr
SwerveSubsystem::SysIdSteerMk4iQuasistaticTorque(frc2::sysid::Direction dir) {
  return frc2::cmd::Sequence(
             frc2::cmd::RunOnce([] { ctre::phoenix6::SignalLogger::Start(); }),
             steerMk4iTorqueSysid.Quasistatic(dir))
      .WithName("Steer Mk4i Quasistatic Torque");
}

frc2::CommandPtr
SwerveSubsystem::SysIdSteerMk4iDynamicTorque(frc2::sysid::Direction dir) {
  return frc2::cmd::Sequence(
             frc2::cmd::RunOnce([] { ctre::phoenix6::SignalLogger::Start(); }),
             steerMk4iTorqueSysid.Dynamic(dir))
      .WithName("Steer Mk4i Dynamic Torque");
}

frc2::CommandPtr
SwerveSubsystem::SysIdSteerMk4nQuasistaticTorque(frc2::sysid::Direction dir) {
  return frc2::cmd::Sequence(
             frc2::cmd::RunOnce([] { ctre::phoenix6::SignalLogger::Start(); }),
             steerMk4nTorqueSysid.Quasistatic(dir))
      .WithName("Steer Mk4n Quasistatic Torque");
}

frc2::CommandPtr
SwerveSubsystem::SysIdSteerMk4nDynamicTorque(frc2::sysid::Direction dir) {
  return frc2::cmd::Sequence(
             frc2::cmd::RunOnce([] { ctre::phoenix6::SignalLogger::Start(); }),
             steerMk4nTorqueSysid.Dynamic(dir))
      .WithName("Steer Mk4n Dynamic Torque");
}

frc2::CommandPtr
SwerveSubsystem::SysIdDriveQuasistaticTorque(frc2::sysid::Direction dir) {
  return frc2::cmd::Sequence(
             frc2::cmd::RunOnce([] { ctre::phoenix6::SignalLogger::Start(); },
                                {this}),
             frc2::cmd::Parallel(PointWheelsToAngle([] { return 0_rad; }),
                                 frc2::cmd::Wait(1_s)),
             driveTorqueSysid.Quasistatic(dir),
             PointWheelsToAngle([] { return 0_rad; }))
      .WithName("Drive Quasistatic Torque");
}

frc2::CommandPtr
SwerveSubsystem::SysIdDriveDynamicTorque(frc2::sysid::Direction dir) {
  return frc2::cmd::Sequence(
             frc2::cmd::RunOnce([] { ctre::phoenix6::SignalLogger::Start(); },
                                {this}),
             frc2::cmd::Parallel(PointWheelsToAngle([] { return 0_rad; }),
                                 frc2::cmd::Wait(1_s)),
             driveTorqueSysid.Dynamic(dir),
             PointWheelsToAngle([] { return 0_rad; }))
      .WithName("Drive Dynamic Torque");
}

frc2::CommandPtr
SwerveSubsystem::SysIdSteerMk4iQuasistaticVoltage(frc2::sysid::Direction dir) {
  return frc2::cmd::Sequence(
             frc2::cmd::RunOnce([] { ctre::phoenix6::SignalLogger::Start(); },
                                {this}),
             steerMk4iVoltageSysid.Quasistatic(dir))
      .WithName("Steer Mk4i Quasistatic Voltage");
}

frc2::CommandPtr
SwerveSubsystem::SysIdSteerMk4iDynamicVoltage(frc2::sysid::Direction dir) {
  return frc2::cmd::Sequence(
             frc2::cmd::RunOnce([] { ctre::phoenix6::SignalLogger::Start(); },
                                {this}),
             steerMk4iVoltageSysid.Dynamic(dir))
      .WithName("Steer Mk4i Dynamic Voltage");
}

frc2::CommandPtr
SwerveSubsystem::SysIdSteerMk4nQuasistaticVoltage(frc2::sysid::Direction dir) {
  return frc2::cmd::Sequence(
             frc2::cmd::RunOnce([] { ctre::phoenix6::SignalLogger::Start(); },
                                {this}),
             steerMk4nVoltageSysid.Quasistatic(dir))
      .WithName("Steer Mk4n Quasistatic Voltage");
}

frc2::CommandPtr
SwerveSubsystem::SysIdSteerMk4nDynamicVoltage(frc2::sysid::Direction dir) {
  return frc2::cmd::Sequence(
             frc2::cmd::RunOnce([] { ctre::phoenix6::SignalLogger::Start(); },
                                {this}),
             steerMk4nVoltageSysid.Dynamic(dir))
      .WithName("Steer Mk4n Dynamic Voltage");
}

frc2::CommandPtr
SwerveSubsystem::SysIdDriveQuasistaticVoltage(frc2::sysid::Direction dir) {
  return frc2::cmd::Sequence(
             frc2::cmd::RunOnce([] { ctre::phoenix6::SignalLogger::Start(); },
                                {this}),
             frc2::cmd::Parallel(PointWheelsToAngle([] { return 0_rad; }),
                                 frc2::cmd::Wait(1_s)),
             driveVoltageSysid.Quasistatic(dir),
             PointWheelsToAngle([] { return 0_rad; }))
      .WithName("Drive Quasistatic Voltage");
}

frc2::CommandPtr
SwerveSubsystem::SysIdDriveDynamicVoltage(frc2::sysid::Direction dir) {
  return frc2::cmd::Sequence(
             frc2::cmd::RunOnce([] { ctre::phoenix6::SignalLogger::Start(); },
                                {this}),
             frc2::cmd::Parallel(PointWheelsToAngle([] { return 0_rad; }),
                                 frc2::cmd::Wait(1_s)),
             driveVoltageSysid.Dynamic(dir),
             PointWheelsToAngle([] { return 0_rad; }))
      .WithName("Drive Dynamic Voltage");
}

frc2::CommandPtr SwerveSubsystem::WheelRadius(frc2::sysid::Direction dir) {
  return frc2::cmd::Sequence(
             frc2::cmd::RunOnce(
                 [this] {
                   wheelRadData.lastGyroYaw = swerveDrive.GetYawFromImu();
                   wheelRadData.accumGyroYaw = 0_rad;
                   wheelRadData.startWheelPositions =
                       swerveDrive.GetModuleDriveOutputShaftPositions();
                   wheelRadData.omegaLimiter.Reset(0_rad_per_s);
                   wheelRadData.effectiveWheelRadius = 0_in;
                 },
                 {this}),
             frc2::cmd::RunEnd(
                 [this, dir] {
                   double dirMulti = 1.0;
                   if (dir == frc2::sysid::Direction::kReverse) {
                     dirMulti = -1.0;
                   }
                   units::radian_t currentYaw = swerveDrive.GetYawFromImu();
                   swerveDrive.Drive(0_mps, 0_mps,
                                     wheelRadData.omegaLimiter.Calculate(
                                         1_rad_per_s * dirMulti),
                                     true);
                   wheelRadData.accumGyroYaw +=
                       frc::AngleModulus(currentYaw - wheelRadData.lastGyroYaw);
                   wheelRadData.lastGyroYaw = currentYaw;
                   units::radian_t avgWheelPos = 0.0_rad;
                   std::array<units::radian_t, 4> currentPositions;
                   currentPositions =
                       swerveDrive.GetModuleDriveOutputShaftPositions();
                   for (int i = 0; i < 4; i++) {
                     avgWheelPos +=
                         units::math::abs(currentPositions[i] -
                                          wheelRadData.startWheelPositions[i]);
                   }
                   avgWheelPos /= 4.0;
                   wheelRadData.effectiveWheelRadius =
                       (wheelRadData.accumGyroYaw *
                        consts::swerve::physical::DRIVEBASE_RADIUS) /
                       avgWheelPos;
                 },
                 [this] {
                   swerveDrive.Drive(0_mps, 0_mps, 0_rad_per_s, true);
                   fmt::print("WHEEL RADIUS: {}\n\n\n\n\n",
                              wheelRadData.effectiveWheelRadius
                                  .convert<units::inches>()
                                  .value());
                 },
                 {this}))
      .WithName("Wheel Radius Calculation");
}
