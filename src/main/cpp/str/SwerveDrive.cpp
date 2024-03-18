// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "str/SwerveDrive.h"

using namespace str;

SwerveDrive::SwerveDrive() {
  for(std::size_t i = 0; i < modules.size(); i++) {
    const auto& modSigs = modules[i].GetSignals();
    allSignals[(i * 8) + 0] = modSigs[0];
    allSignals[(i * 8) + 1] = modSigs[1];
    allSignals[(i * 8) + 2] = modSigs[2];
    allSignals[(i * 8) + 3] = modSigs[3];
    allSignals[(i * 8) + 4] = modSigs[4];
    allSignals[(i * 8) + 5] = modSigs[5];
    allSignals[(i * 8) + 6] = modSigs[6];
    allSignals[(i * 8) + 7] = modSigs[7];
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

  if(!imu.OptimizeBusUtilization().IsOK()) {
    fmt::print("Failed to optimize bus signals for imu!\n");
  }
}

void SwerveDrive::Drive(units::meters_per_second_t xVel, units::meters_per_second_t yVel, units::radians_per_second_t omega, bool fieldRelative) {

  units::second_t now = frc::Timer::GetFPGATimestamp();
  units::second_t loopTime = now - lastDriveLoopTime;

  frc::ChassisSpeeds speedsToSend;
  if(fieldRelative) {
    speedsToSend = frc::ChassisSpeeds::FromFieldRelativeSpeeds(xVel, yVel, omega, GetRotationFromImu().Z());
  }
  else {
    speedsToSend.vx = xVel;
    speedsToSend.vy = yVel;
    speedsToSend.omega = omega;
  }

  speedsToSend = frc::ChassisSpeeds::Discretize(speedsToSend, loopTime);

  SetModuleStates(consts::swerve::physical::KINEMATICS.ToSwerveModuleStates(speedsToSend));

  lastDriveLoopTime = now;
}

frc::Pose2d SwerveDrive::GetOdomPose() const {
  return odom.GetPose();
}

frc::Pose2d SwerveDrive::GetPose() const {
  return poseEstimator.GetEstimatedPosition();
}

frc::Rotation3d SwerveDrive::GetRotationFromImu() {
  return imu.GetRotation3d();
}

void SwerveDrive::SetModuleStates(const std::array<frc::SwerveModuleState, 4>& desiredStates) {
  wpi::array<frc::SwerveModuleState, 4> desaturatedStates = desiredStates;
  frc::SwerveDriveKinematics<4>::DesaturateWheelSpeeds(&desaturatedStates, consts::swerve::physical::DRIVE_MAX_SPEED);
  int i = 0;
  for(auto& mod : modules) {
    mod.GoToState(desaturatedStates[i]);
    i++;
  }
  desiredStatesPub.Set(desaturatedStates);
}

void SwerveDrive::UpdateSwerveOdom() {
  ctre::phoenix::StatusCode status = ctre::phoenix6::BaseStatusSignal::WaitForAll(2.0 / 250_Hz, allSignals);

  // if(!status.IsOK()) {
  //   fmt::print("Error updating swerve odom! Error was: {}\n", status.GetName());
  // }

  int i = 0;
  for(auto& mod : modules) {
    modulePositions[i] = mod.GetCurrentPosition(false);
    moduleStates[i] = mod.GetCurrentState();
    i++;
  }

  units::radian_t yawLatencyComped = ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(imu.GetYaw(), imu.GetAngularVelocityZWorld());
  poseEstimator.Update(frc::Rotation2d{yawLatencyComped}, modulePositions);
  odom.Update(frc::Rotation2d{yawLatencyComped}, modulePositions);
}

void SwerveDrive::UpdateNTEntries() {
  currentStatesPub.Set(moduleStates);
  currentPositionsPub.Set(modulePositions);
  odomPosePub.Set(GetOdomPose());
  estimatorPub.Set(GetPose());
}

void SwerveDrive::SimulationPeriodic() {
  units::second_t now = frc::Timer::GetFPGATimestamp();
  units::second_t loopTime = now - lastSimLoopTime;

  std::array<frc::SwerveModuleState, 4> simState;
  int i = 0;
  for(auto& swerveModule : modules) {
    simState[i] = swerveModule.UpdateSimulation(loopTime, frc::RobotController::GetBatteryVoltage());
    i++;
  }

  simStatesPub.Set(simState);

  units::radians_per_second_t omega = consts::swerve::physical::KINEMATICS.ToChassisSpeeds(simState).omega;
  units::radian_t angleChange = omega * loopTime;

  lastSimAngle = lastSimAngle + frc::Rotation2d{angleChange};
  imuSimState.SetRawYaw(lastSimAngle.Degrees());

  lastSimLoopTime = now;
}

units::ampere_t SwerveDrive::GetSimulatedCurrentDraw() const {
  units::ampere_t totalCurrent = 0_A;
  for(const auto& swerveModule : modules) {
    totalCurrent += swerveModule.GetSimulatedCurrentDraw();
  }
  return totalCurrent;
}

void SwerveDrive::SetCharacterizationTorqueSteer(units::volt_t torqueAmps) {
  modules[0].SetSteerToTorque(torqueAmps);
}

void SwerveDrive::SetCharacterizationTorqueDrive(units::volt_t torqueAmps) {
  modules[0].SetDriveToTorque(torqueAmps);
  modules[1].SetDriveToTorque(torqueAmps);
  modules[2].SetDriveToTorque(torqueAmps);
  modules[3].SetDriveToTorque(torqueAmps);
}

void SwerveDrive::SetCharacterizationVoltageSteer(units::volt_t volts) {
  modules[0].SetSteerToVoltage(volts);
}

void SwerveDrive::SetCharacterizationVoltageDrive(units::volt_t volts) {
  modules[0].SetDriveToVoltage(volts);
  modules[1].SetDriveToVoltage(volts);
  modules[2].SetDriveToVoltage(volts);
  modules[3].SetDriveToVoltage(volts);
}

void SwerveDrive::LogSteerTorque(frc::sysid::SysIdRoutineLog* log) {
  log->Motor("swerve-steer")
    .voltage(units::volt_t{allSignals[5]->GetValueAsDouble()})
    .position(units::turn_t{allSignals[2]->GetValueAsDouble()})
    .velocity(units::turns_per_second_t{allSignals[3]->GetValueAsDouble()});
}

void SwerveDrive::LogDriveTorque(frc::sysid::SysIdRoutineLog* log) {
  log->Motor("swerve-drive")
    .voltage(units::volt_t{allSignals[4]->GetValueAsDouble()})
    .position(units::turn_t{allSignals[0]->GetValueAsDouble()})
    .velocity(units::turns_per_second_t{allSignals[1]->GetValueAsDouble()});
}

void SwerveDrive::LogSteerVoltage(frc::sysid::SysIdRoutineLog* log) {
  log->Motor("swerve-steer")
    .voltage(units::volt_t{allSignals[7]->GetValueAsDouble()})
    .position(units::turn_t{allSignals[2]->GetValueAsDouble()})
    .velocity(units::turns_per_second_t{allSignals[3]->GetValueAsDouble()});
}

void SwerveDrive::LogDriveVoltage(frc::sysid::SysIdRoutineLog* log) {
  log->Motor("swerve-drive")
    .voltage(units::volt_t{allSignals[6]->GetValueAsDouble()})
    .position(units::turn_t{allSignals[0]->GetValueAsDouble()})
    .velocity(units::turns_per_second_t{allSignals[1]->GetValueAsDouble()});
}
