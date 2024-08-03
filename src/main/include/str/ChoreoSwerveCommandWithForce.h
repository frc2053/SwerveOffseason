// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <choreo/lib/Choreo.h>
#include <frc/Timer.h>
#include <frc2/command/CommandHelper.h>
#include <units/torque.h>

#include <functional>

#include "choreo/lib/ChoreoTrajectory.h"

namespace str {
/**
 * A frc2::Command that controls a swerve drivetrain using ChoreoTrajectories
 */
class ChoreoSwerveCommandWithForce
    : public frc2::CommandHelper<frc2::Command, ChoreoSwerveCommandWithForce> {
public:
  /**
   * Creates a new ChoreoSwerveCommand that controls a swerve drivetrain
   *
   * @param trajectory the ChoreoTrajectory to follow
   * @param poseSupplier a function that supplies the current pose of the robot
   * @param controller a function that consumes a pose and the current
   *  trajectory state, and supplies back robot relative ChassisSpeeds
   * @param outputChassisSpeeds a function that consumes robot relative
   *  ChassisSpeeds
   * @param outputXForce a function that the modules predicted force in the
   * field relative x direction
   * @param outputYForce a function that the modules predicted force in the
   * field relative ys direction
   * @param mirrorTrajectory If this returns true, the path will be mirrored to
   * the opposite side, while keeping the same coordinate system origin. This
   * will be called every loop during the command.
   * @param requirements subsystem requirements
   */
  ChoreoSwerveCommandWithForce(
      choreolib::ChoreoTrajectory trajectory,
      std::function<frc::Pose2d()> poseSupplier,
      choreolib::ChoreoControllerFunction controller,
      std::function<void(frc::ChassisSpeeds)> outputChassisSpeeds,
      std::function<void(std::array<units::newton_t, 4>)> outputXForce,
      std::function<void(std::array<units::newton_t, 4>)> outputYForce,
      std::function<bool(void)> mirrorTrajectory,
      frc2::Requirements requirements = {});

  /// Runs once before the first call to Execute()
  void Initialize() override;

  /// Runs every robot periodic loop while the command is running.
  void Execute() override;

  /// Runs once after IsFinished() returns true
  void End(bool interrupted) override;

  /// Command will end once this returns true
  bool IsFinished() override;

private:
  frc::Timer m_timer;
  choreolib::ChoreoTrajectory m_traj;
  std::function<frc::Pose2d()> m_pose;
  choreolib::ChoreoControllerFunction m_controller;
  std::function<void(frc::ChassisSpeeds)> m_outputChassisSpeeds;
  std::function<void(std::array<units::newton_t, 4>)> m_outputXForce;
  std::function<void(std::array<units::newton_t, 4>)> m_outputYForce;
  std::function<bool(void)> m_mirrorTrajectory;
};
} // namespace str
