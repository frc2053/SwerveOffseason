// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/TimedRobot.h>
#include <frc/simulation/BatterySim.h>
#include <frc2/command/CommandPtr.h>
#include <networktables/DoubleTopic.h>

#include <optional>

#include "RobotContainer.h"
#include "constants/Constants.h"
#include "str/SwerveModule.h"

class Robot : public frc::TimedRobot {
public:
  Robot() : frc::TimedRobot(consts::LOOP_PERIOD) {}
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
  frc2::Command *m_autonomousCommand = nullptr;

  RobotContainer m_container;

  units::second_t lastTotalLoopTime;
  nt::DoublePublisher loopTimePub{nt::NetworkTableInstance::GetDefault()
                                      .GetTable("Looptime")
                                      ->GetDoubleTopic("RobotPeriodicLoopRate")
                                      .Publish()};
};
