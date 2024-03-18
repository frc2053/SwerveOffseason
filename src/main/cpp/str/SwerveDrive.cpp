// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "str/SwerveDrive.h"

using namespace str;

SwerveDrive::SwerveDrive() {
  for(int i = 0; modules.size(); i++) {
    const auto& modSigs = modules[i].GetSignals();
    allSignals[i * 6 + 0] = modSigs[0];
    allSignals[i * 6 + 1] = modSigs[1];
    allSignals[i * 6 + 2] = modSigs[2];
    allSignals[i * 6 + 3] = modSigs[3];
    allSignals[i * 6 + 4] = modSigs[4];
    allSignals[i * 6 + 5] = modSigs[5];
  }

  allSignals[allSignals.size() - 2] = &imu.GetYaw();
  allSignals[allSignals.size() - 1] = &imu.GetAngularVelocityZWorld();

  for(const auto& sig : allSignals) {
    sig->SetUpdateFrequency(250_Hz);
  }

  for(auto& mod : modules) {
    if(!mod.OptimizeBusSignals()) {
      fmt::print("Failed to optimize bus signals for {}\n", mod.GetName());
    }
  }
}

void SwerveDrive::UpdateSwerveOdom() {
  ctre::phoenix::StatusCode status = ctre::phoenix6::BaseStatusSignal::WaitForAll(2.0 / 250_Hz, allSignals);

  for(auto& mod : modules) {
    mod.GetCurrentPosition(false);
  }
}

void SwerveDrive::SimulationPeriodic() {
  units::second_t now = units::microsecond_t{WPI_Now()};
  units::second_t loopTime = now - lastLoopTime;
  for(auto& swerveModule : modules) {
    swerveModule.UpdateSimulation(loopTime, frc::RobotController::GetBatteryVoltage());
  }
  lastLoopTime = now;
}

units::ampere_t SwerveDrive::GetSimulatedCurrentDraw() const {
  units::ampere_t totalCurrent = 0_A;
  for(const auto& swerveModule : modules) {
    totalCurrent += swerveModule.GetSimulatedCurrentDraw();
  }
  return totalCurrent;
}
