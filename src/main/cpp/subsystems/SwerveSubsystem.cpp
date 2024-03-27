// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveSubsystem.h"
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <str/DriverstationUtils.h>

SwerveSubsystem::SwerveSubsystem() {
  frc::SmartDashboard::PutData(this);
  SetupPathplanner();
}

void SwerveSubsystem::UpdateSwerveOdom()
{
  swerveDrive.UpdateSwerveOdom();
}

// This method will be called once per scheduler run
void SwerveSubsystem::Periodic() {
  swerveDrive.UpdateNTEntries();
}

void SwerveSubsystem::SimulationPeriodic() {
  swerveDrive.SimulationPeriodic();
}

units::ampere_t SwerveSubsystem::GetSimulatedCurrentDraw() const {
  return swerveDrive.GetSimulatedCurrentDraw();
}

frc2::CommandPtr SwerveSubsystem::PointWheelsToAngle(std::function<units::radian_t()> wheelAngle) {
  return frc2::cmd::RunOnce([this, wheelAngle] {

    std::array<frc::SwerveModuleState, 4> states;
    for(auto& state : states) {
      state.angle = wheelAngle();
      state.speed = 0_mps;
    }

    swerveDrive.SetModuleStates(states, false);
  }, {this}).WithName("Point Wheels At Angle");
}

void SwerveSubsystem::SetupPathplanner() {
  pathplanner::AutoBuilder::configureHolonomic(
    [this] {
      return swerveDrive.GetPose();
    },
    [this](frc::Pose2d resetPose) {
      return swerveDrive.ResetPose(resetPose);
    },
    [this] {
      return swerveDrive.GetRobotRelativeSpeeds();
    },
    [this](frc::ChassisSpeeds robotRelativeOutput) {
      swerveDrive.DriveRobotRelative(robotRelativeOutput);
    },
    consts::swerve::pathplanning::PATH_CONFIG,
    [] { return str::IsOnRed(); },
    this
  );
}

frc2::CommandPtr SwerveSubsystem::Drive(std::function<units::meters_per_second_t()> xVel, std::function<units::meters_per_second_t()> yVel, std::function<units::radians_per_second_t()> omega, bool fieldRelative) {
  return frc2::cmd::Run([this, xVel, yVel, omega, fieldRelative] {
    swerveDrive.Drive(xVel(), yVel(), omega(), fieldRelative);
  }, {this}).WithName("Drive Command");
}

frc2::CommandPtr SwerveSubsystem::AlignToAmp() {
  auto alignToAmpPath = pathplanner::PathPlannerPath::fromChoreoTrajectory("AlignToAmp");
  return pathplanner::AutoBuilder::pathfindThenFollowPath(alignToAmpPath, consts::swerve::pathplanning::constraints);
}

frc2::CommandPtr SwerveSubsystem::SysIdSteerQuasistaticTorque(frc2::sysid::Direction dir) {
  return frc2::cmd::Sequence(
    frc2::cmd::RunOnce([] { 
      ctre::phoenix6::SignalLogger::Start();
    }),
    steerTorqueSysid.Quasistatic(dir)
  ).WithName("Steer Quasistatic Torque");
}

frc2::CommandPtr SwerveSubsystem::SysIdSteerDynamicTorque(frc2::sysid::Direction dir) {
  return frc2::cmd::Sequence(
    frc2::cmd::RunOnce([] { 
      ctre::phoenix6::SignalLogger::Start();
    }),
    steerTorqueSysid.Dynamic(dir)
  ).WithName("Steer Dynamic Torque");}

frc2::CommandPtr SwerveSubsystem::SysIdDriveQuasistaticTorque(frc2::sysid::Direction dir) {
  return frc2::cmd::Sequence(
    frc2::cmd::RunOnce([] { 
      ctre::phoenix6::SignalLogger::Start();
    }, {this}),
    frc2::cmd::Parallel(
      PointWheelsToAngle([] { return 0_rad; }),
      frc2::cmd::Wait(1_s)
    ),
    driveTorqueSysid.Quasistatic(dir),
    PointWheelsToAngle([] { return 0_rad; })
  ).WithName("Drive Quasistatic Torque");}

frc2::CommandPtr SwerveSubsystem::SysIdDriveDynamicTorque(frc2::sysid::Direction dir) {
  return frc2::cmd::Sequence(
    frc2::cmd::RunOnce([] { 
      ctre::phoenix6::SignalLogger::Start();
    }, {this}),
    frc2::cmd::Parallel(
      PointWheelsToAngle([] { return 0_rad; }),
      frc2::cmd::Wait(1_s)
    ),
    driveTorqueSysid.Dynamic(dir),
    PointWheelsToAngle([] { return 0_rad; })
  ).WithName("Drive Dynamic Torque");}

frc2::CommandPtr SwerveSubsystem::SysIdSteerQuasistaticVoltage(frc2::sysid::Direction dir) {
  return frc2::cmd::Sequence(
    frc2::cmd::RunOnce([] {
      ctre::phoenix6::SignalLogger::Start();
    }, {this}),
    steerVoltageSysid.Quasistatic(dir)
  ).WithName("Steer Quasistatic Voltage");
}

frc2::CommandPtr SwerveSubsystem::SysIdSteerDynamicVoltage(frc2::sysid::Direction dir) {
  return frc2::cmd::Sequence(
    frc2::cmd::RunOnce([] {
      ctre::phoenix6::SignalLogger::Start();
    }, {this}),
    steerVoltageSysid.Dynamic(dir)
  ).WithName("Steer Quasistatic Voltage");
}

frc2::CommandPtr SwerveSubsystem::SysIdDriveQuasistaticVoltage(frc2::sysid::Direction dir) {
  return frc2::cmd::Sequence(
    frc2::cmd::RunOnce([] { 
      ctre::phoenix6::SignalLogger::Start();
    }, {this}),
    frc2::cmd::Parallel(
      PointWheelsToAngle([] { return 0_rad; }),
      frc2::cmd::Wait(1_s)
    ),
    driveVoltageSysid.Quasistatic(dir),
    PointWheelsToAngle([] { return 0_rad; })
  ).WithName("Drive Quasistatic Voltage");
}

frc2::CommandPtr SwerveSubsystem::SysIdDriveDynamicVoltage(frc2::sysid::Direction dir) {
  return frc2::cmd::Sequence(
    frc2::cmd::RunOnce([] { 
      ctre::phoenix6::SignalLogger::Start();
    }, {this}),
    frc2::cmd::Parallel(
      PointWheelsToAngle([] { return 0_rad; }),
      frc2::cmd::Wait(1_s)
    ),
    driveVoltageSysid.Dynamic(dir),
    PointWheelsToAngle([] { return 0_rad; })
  ).WithName("Drive Dynamic Voltage");
}