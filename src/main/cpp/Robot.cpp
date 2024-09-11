// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "Robot.h"

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/simulation/RoboRioSim.h>
#include <frc2/command/CommandScheduler.h>
#include "frc/Alert.h"

#include "str/DataUtils.h"

void Robot::RobotInit() {
  str::DataUtils::LogGitInfo();
  frc2::CommandScheduler::GetInstance().SetPeriod(consts::LOOP_PERIOD);
  frc::DataLogManager::Start();
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());
  AddPeriodic([this] { m_container.GetSwerveSubsystem().UpdateSwerveOdom(); },
              consts::SWERVE_ODOM_LOOP_PERIOD, 2_ms);
}

void Robot::RobotPeriodic() {
  units::second_t now = frc::Timer::GetFPGATimestamp();
  units::second_t loopTime = now - lastTotalLoopTime;
  loopTimePub.Set((1 / loopTime).value());

  frc2::CommandScheduler::GetInstance().Run();
  m_container.GetNoteVisualizer().Periodic();
  //UpdateVision();

  lastTotalLoopTime = now;
}

void Robot::UpdateVision() {
  // auto visionEstimates = m_container.GetVision().GetCameraEstimatedPoses();
  // auto stdDevs = m_container.GetVision().GetPoseStdDevs(visionEstimates);
  // int i = 0;
  // for (const auto &est : visionEstimates) {
  //   if (est.has_value()) {
  //     // m_container.GetSwerveSubsystem().AddVisionMeasurement(est.value().estimatedPose.ToPose2d(),
  //     // est.value().timestamp, stdDevs[i].value());
  //   }
  //   i++;
  // }
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand != nullptr) {
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

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::TestInit() { frc2::CommandScheduler::GetInstance().CancelAll(); }

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {
  // FIXME: Current simulation is VERY wrong
  // units::volt_t battVoltage =
  // frc::sim::BatterySim::Calculate({m_container.GetSwerveSubsystem().GetSimulatedCurrentDraw()});
  // frc::sim::RoboRioSim::SetVInVoltage(battVoltage);

  // m_container.GetVision().SimulationPeriodic(
  //     m_container.GetSwerveSubsystem().GetOdomPose());
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
