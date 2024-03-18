// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveSubsystem.h"
#include <frc2/command/Commands.h>

SwerveSubsystem::SwerveSubsystem() {
}

void SwerveSubsystem::UpdateSwerveOdom()
{
}

// This method will be called once per scheduler run
void SwerveSubsystem::Periodic() {
}

void SwerveSubsystem::SimulationPeriodic() {
  swerveDrive.SimulationPeriodic();
}

units::ampere_t SwerveSubsystem::GetSimulatedCurrentDraw() const {
  return swerveDrive.GetSimulatedCurrentDraw();
}

frc2::CommandPtr SwerveSubsystem::SysIdSteerQuasistatic(frc2::sysid::Direction dir) {
  return steerTorqueSysid.Quasistatic(dir).BeforeStarting([] { ctre::phoenix6::SignalLogger::Start(); } );
}

frc2::CommandPtr SwerveSubsystem::SysIdSteerDynamic(frc2::sysid::Direction dir) {
  return steerTorqueSysid.Dynamic(dir).BeforeStarting([] { ctre::phoenix6::SignalLogger::Start(); } );
}