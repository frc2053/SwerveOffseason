// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "str/ChoreoSwerveCommandWithForce.h"

#include <frc/DriverStation.h>
#include <frc/controller/PIDController.h>

#include <utility>

using namespace str;

ChoreoSwerveCommandWithForce::ChoreoSwerveCommandWithForce(
    choreolib::ChoreoTrajectory trajectory,
    std::function<frc::Pose2d()> poseSupplier,
    choreolib::ChoreoControllerFunction controller,
    std::function<void(frc::ChassisSpeeds)> outputChassisSpeeds,
    std::function<void(std::array<units::newton_t, 4>)> outputXForce,
    std::function<void(std::array<units::newton_t, 4>)> outputYForce,
    std::function<bool(void)> mirrorTrajectory, frc2::Requirements requirements)
    : m_traj(std::move(trajectory)), m_pose(std::move(poseSupplier)),
      m_controller(std::move(controller)),
      m_outputChassisSpeeds(std::move(outputChassisSpeeds)),
      m_outputXForce(std::move(outputXForce)),
      m_outputYForce(std::move(outputYForce)),
      m_mirrorTrajectory(std::move(mirrorTrajectory)) {
  AddRequirements(requirements);
}

// Called when the command is initially scheduled.
void ChoreoSwerveCommandWithForce::Initialize() { m_timer.Restart(); }

// Called repeatedly when this Command is scheduled to run
void ChoreoSwerveCommandWithForce::Execute() {
  units::second_t currentTrajTime = m_timer.Get();
  bool mirror = m_mirrorTrajectory();
  choreolib::ChoreoTrajectoryState currentState =
      m_traj.Sample(currentTrajTime, mirror);
  m_outputChassisSpeeds(m_controller(m_pose(), currentState));
  m_outputXForce(currentState.moduleForcesX);
  m_outputYForce(currentState.moduleForcesY);
}

// Called once the command ends or is interrupted.
void ChoreoSwerveCommandWithForce::End(bool interrupted) {
  m_outputChassisSpeeds(frc::ChassisSpeeds{});
  m_outputXForce({});
  m_outputYForce({});
}

// Returns true when the command should end.
bool ChoreoSwerveCommandWithForce::IsFinished() {
  return m_timer.HasElapsed(m_traj.GetTotalTime());
}
