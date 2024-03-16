// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveSubsystem.h"

SwerveSubsystem::SwerveSubsystem() {
  allSignals = testModule.GetSignals();

  for(const auto& sig : allSignals) {
    sig->SetUpdateFrequency(250_Hz);
  }
  if(!testModule.OptimizeBusSignals()) {
    fmt::print("Failed to optimize bus signals for {}\n", testModule.GetName());
  }
}

void SwerveSubsystem::UpdateSwerveOdom()
{
  ctre::phoenix::StatusCode status = ctre::phoenix6::BaseStatusSignal::WaitForAll(2.0 / 250_Hz, allSignals);
}

// This method will be called once per scheduler run
void SwerveSubsystem::Periodic() {
}

void SwerveSubsystem::SimulationPeriodic() {
  units::second_t now = units::microsecond_t{WPI_Now()};
  units::second_t loopTime = now - lastLoopTime;
  testModule.UpdateSimulation(loopTime, frc::RobotController::GetBatteryVoltage());
  lastLoopTime = now;
}

units::ampere_t SwerveSubsystem::GetCurrentDraw() const {
  return testModule.GetSimulatedCurrentDraw();
}

frc2::CommandPtr SwerveSubsystem::SysIdSteerQuasistatic(frc2::sysid::Direction dir) {
  return steerTorqueSysid.Quasistatic(dir).BeforeStarting([] { ctre::phoenix6::SignalLogger::Start(); } );
}

frc2::CommandPtr SwerveSubsystem::SysIdSteerDynamic(frc2::sysid::Direction dir) {
  return steerTorqueSysid.Dynamic(dir).BeforeStarting([] { ctre::phoenix6::SignalLogger::Start(); } );
}