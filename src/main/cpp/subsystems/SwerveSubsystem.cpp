// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveSubsystem.h"
#include <frc2/command/Commands.h>

SwerveSubsystem::SwerveSubsystem() {
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

frc2::CommandPtr SwerveSubsystem::PointWheelsToZero() {
  return frc2::cmd::RunOnce([this] {

    std::array<frc::SwerveModuleState, 4> states;
    for(auto& state : states) {
      state.angle = 0_rad;
      state.speed = 0_mps;
    }

    swerveDrive.SetModuleStates(states);
  }, {this}).AndThen(frc2::cmd::Wait(1_s));
}

frc2::CommandPtr SwerveSubsystem::Drive(std::function<units::meters_per_second_t()> xVel, std::function<units::meters_per_second_t()> yVel, std::function<units::radians_per_second_t()> omega, bool fieldRelative) {
  return frc2::cmd::Run([this, xVel, yVel, omega, fieldRelative] {
    swerveDrive.Drive(xVel(), yVel(), omega(), fieldRelative);
  }, {this});
}

frc2::CommandPtr SwerveSubsystem::SysIdSteerQuasistatic(frc2::sysid::Direction dir) {
  return steerTorqueSysid.Quasistatic(dir).BeforeStarting([] { ctre::phoenix6::SignalLogger::Start(); } );
}

frc2::CommandPtr SwerveSubsystem::SysIdSteerDynamic(frc2::sysid::Direction dir) {
  return steerTorqueSysid.Dynamic(dir).BeforeStarting([] { ctre::phoenix6::SignalLogger::Start(); } );
}

frc2::CommandPtr SwerveSubsystem::SysIdDriveQuasistatic(frc2::sysid::Direction dir) {
  return driveTorqueSysid.Quasistatic(dir).BeforeStarting([] { ctre::phoenix6::SignalLogger::Start(); } );
}

frc2::CommandPtr SwerveSubsystem::SysIdDriveDynamic(frc2::sysid::Direction dir) {
  return driveTorqueSysid.Dynamic(dir).BeforeStarting([] { ctre::phoenix6::SignalLogger::Start(); } ).BeforeStarting(PointWheelsToZero());
}