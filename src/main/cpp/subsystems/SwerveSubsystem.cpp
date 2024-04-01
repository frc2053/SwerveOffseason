// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveSubsystem.h"
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <str/DriverstationUtils.h>

SwerveSubsystem::SwerveSubsystem() {
  SetName("SwerveSubsystem");
  frc::SmartDashboard::PutData(this);
  SetupPathplanner();
}

void SwerveSubsystem::UpdateSwerveOdom()
{
  swerveDrive.UpdateSwerveOdom();
}

void SwerveSubsystem::AddVisionMeasurement(const frc::Pose2d& visionMeasurement, units::second_t timestamp, const Eigen::Vector3d& stdDevs) {
  swerveDrive.AddVisionMeasurement(visionMeasurement, timestamp, stdDevs);
}

// This method will be called once per scheduler run
void SwerveSubsystem::Periodic() {
  swerveDrive.UpdateNTEntries();
}

void SwerveSubsystem::SimulationPeriodic() {
  swerveDrive.SimulationPeriodic();
}

frc::Pose2d SwerveSubsystem::GetOdomPose() {
  return swerveDrive.GetOdomPose();
}

frc::Pose2d SwerveSubsystem::GetRobotPose() {
  return swerveDrive.GetPose();
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

frc2::CommandPtr SwerveSubsystem::XPattern() {
  return frc2::cmd::Run([this] {
    std::array<frc::SwerveModuleState, 4> states;

    states[0].angle = 45_deg;
    states[1].angle = -45_deg;
    states[2].angle = -45_deg;
    states[3].angle = 45_deg;

    swerveDrive.SetModuleStates(states, true);
  }, {this}).WithName("X Pattern");
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
  return pathplanner::AutoBuilder::pathfindThenFollowPath(alignToAmpPath, consts::swerve::pathplanning::constraints).WithName("AlignToAmp");
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

frc2::CommandPtr SwerveSubsystem::WheelRadius(frc2::sysid::Direction dir) {
  return frc2::cmd::Sequence(
    frc2::cmd::RunOnce([this] {
      wheelRadData.lastGyroYaw = swerveDrive.GetYawFromImu();
      wheelRadData.accumGyroYaw = 0_rad;
      wheelRadData.startWheelPositions = swerveDrive.GetModuleDriveOutputShaftPositions();
      wheelRadData.omegaLimiter.Reset(0_rad_per_s);
      wheelRadData.effectiveWheelRadius = 0_in;
    }, 
    {this}
    ),
    frc2::cmd::RunEnd([this, dir] {
        double dirMulti = 1.0;
        if(dir == frc2::sysid::Direction::kReverse) {
          dirMulti = -1.0;
        }
        units::radian_t currentYaw = swerveDrive.GetYawFromImu();
        swerveDrive.Drive(0_mps, 0_mps, wheelRadData.omegaLimiter.Calculate(1_rad_per_s * dirMulti), true);
        wheelRadData.accumGyroYaw += frc::AngleModulus(currentYaw - wheelRadData.lastGyroYaw);
        wheelRadData.lastGyroYaw = currentYaw;
        units::radian_t avgWheelPos = 0.0_rad;
        std::array<units::radian_t, 4> currentPositions;
        currentPositions = swerveDrive.GetModuleDriveOutputShaftPositions();
        for (int i = 0; i < 4; i++) {
          avgWheelPos += units::math::abs(currentPositions[i] -
                                          wheelRadData.startWheelPositions[i]);
        }
        avgWheelPos /= 4.0;
        wheelRadData.effectiveWheelRadius = (wheelRadData.accumGyroYaw * consts::swerve::physical::DRIVEBASE_RADIUS) / avgWheelPos;
    },
    [this] {
      swerveDrive.Drive(0_mps, 0_mps, 0_rad_per_s, true);
      fmt::print("WHEEL RADIUS: {}\n\n\n\n\n", wheelRadData.effectiveWheelRadius.convert<units::inches>().value());
    },
    {this})
  ).WithName("Wheel Radius Calculation");
}