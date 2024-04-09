// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <optional>

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>

#include "RobotContainer.h"
#include "str/SwerveModule.h"
#include <frc/simulation/BatterySim.h>
#include "constants/Constants.h"
#include <networktables/DoubleTopic.h>

class Robot : public frc::TimedRobot {
 public:
  Robot() : frc::TimedRobot(consts::LOOP_PERIOD) {};
  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void DisabledExit() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void AutonomousExit() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TeleopExit() override;
  void TestInit() override;
  void TestPeriodic() override;
  void TestExit() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;
  void UpdateVision();

 private:
  frc2::Command* m_autonomousCommand = nullptr;

  RobotContainer m_container;

  units::second_t lastTotalLoopTime;
  nt::DoublePublisher loopTimePub{nt::NetworkTableInstance::GetDefault().GetTable("Metadata")->GetDoubleTopic("LoopRate").Publish()};
};
