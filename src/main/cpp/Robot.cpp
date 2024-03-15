// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <frc/simulation/RoboRioSim.h>

void Robot::UpdateModule() {
  ctre::phoenix::StatusCode status = ctre::phoenix6::BaseStatusSignal::WaitForAll(2.0 / 250_Hz, allSignals);
}

void Robot::RobotInit() {
  allSignals = testModule.GetSignals();

  for(const auto& sig : allSignals) {
    sig->SetUpdateFrequency(250_Hz);
  }
  if(!testModule.OptimizeBusSignals()) {
    fmt::print("Failed to optimize bus signals for {}\n", testModule.GetName());
  }

  AddPeriodic([this] { UpdateModule(); }, 1 / 250_Hz);
}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }
}

void Robot::TeleopPeriodic() {
  desiredState.angle = units::radian_t{std::atan2(-controller.GetLeftY(), -controller.GetLeftX())};
  desiredState.speed = units::feet_per_second_t{std::hypot(-controller.GetLeftY(), -controller.GetLeftX())} * 15;
  testModule.GoToState(desiredState);
  testModule.GetCurrentState();
  testModule.GetCurrentPosition(false);
}

void Robot::TeleopExit() {}

void Robot::TestInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

void Robot::SimulationInit() {

}

void Robot::SimulationPeriodic() {
  testModule.UpdateSimulation(GetPeriod(), frc::RobotController::GetBatteryVoltage());
  units::volt_t battVoltage = frc::sim::BatterySim::Calculate({testModule.GetSimulatedCurrentDraw()});
  frc::sim::RoboRioSim::SetVInVoltage(battVoltage);
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
