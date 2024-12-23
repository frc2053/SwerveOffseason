// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "str/SwerveDrive.h"

#include <str/DriverstationUtils.h>
#include <frc/DataLogManager.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "constants/Constants.h"
#include "str/Math.h"

using namespace str;

SwerveDrive::SwerveDrive() {
  for (size_t i = 0; i < modules.size(); i++) {
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

  ctre::phoenix6::configs::Pigeon2Configuration imuConfig;
  imuConfig.MountPose.MountPoseRoll =
      consts::swerve::physical::IMU_ROLL_OFFSET.convert<units::degrees>()
          .value();
  imuConfig.MountPose.MountPosePitch =
      consts::swerve::physical::IMU_PITCH_OFFSET.convert<units::degrees>()
          .value();
  imuConfig.MountPose.MountPoseYaw =
      consts::swerve::physical::IMU_YAW_OFFSET.convert<units::degrees>()
          .value();

  if (!imu.GetConfigurator().Apply(imuConfig).IsOK()) {
    frc::DataLogManager::Log("Failed to configure IMU!\n");
  }

  allSignals[allSignals.size() - 2] = &imu.GetYaw();
  allSignals[allSignals.size() - 1] = &imu.GetAngularVelocityZWorld();

  for (const auto& sig : allSignals) {
    sig->SetUpdateFrequency(1 / consts::SWERVE_ODOM_LOOP_PERIOD);
  }

  for (auto& mod : modules) {
    if (!mod.OptimizeBusSignals()) {
      frc::DataLogManager::Log(fmt::format(
          "Failed to optimize bus signals for {}\n", mod.GetName()));
    }
  }

  if (!imu.OptimizeBusUtilization().IsOK()) {
    frc::DataLogManager::Log("Failed to optimize bus signals for imu!\n");
  }

  lastOdomUpdateTime = frc::Timer::GetFPGATimestamp();
  frc::SmartDashboard::PutData(&field);
}

void SwerveDrive::ResetPose(const frc::Pose2d& resetPose) {
  odom.ResetPosition(frc::Rotation2d{GetYawFromImu()}, modulePositions,
                     resetPose);
  poseEstimator.ResetPosition(frc::Rotation2d{GetYawFromImu()}, modulePositions,
                              resetPose);
}

void SwerveDrive::DriveRobotRelative(
    const frc::ChassisSpeeds& robotRelativeSpeeds) {
  Drive(robotRelativeSpeeds.vx, robotRelativeSpeeds.vy,
        robotRelativeSpeeds.omega, false);
}

void SwerveDrive::Drive(units::meters_per_second_t xVel,
                        units::meters_per_second_t yVel,
                        units::radians_per_second_t omega, bool fieldRelative,
                        bool openLoop) {
  frc::ChassisSpeeds speedsToSend;

  frc::Rotation2d rot = poseEstimator.GetEstimatedPosition().Rotation();

  if(frc::DriverStation::IsTeleop()) {
    rot = odom.GetPose().Rotation();
  }
  
  if (fieldRelative) {
    speedsToSend = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        xVel, yVel, omega, rot);
  } else {
    speedsToSend.vx = xVel;
    speedsToSend.vy = yVel;
    speedsToSend.omega = omega;
  }

  speedsToSend =
      frc::ChassisSpeeds::Discretize(speedsToSend, consts::LOOP_PERIOD);

  std::array<frc::SwerveModuleState, 4> states = consts::swerve::physical::KINEMATICS.ToSwerveModuleStates(speedsToSend);

  SetModuleStates(states,
      true, openLoop,
      ConvertModuleForcesToTorqueCurrent(xModuleForce, yModuleForce));
}

frc::ChassisSpeeds SwerveDrive::GetRobotRelativeSpeeds() const {
  return consts::swerve::physical::KINEMATICS.ToChassisSpeeds(moduleStates);
}

frc::Pose2d SwerveDrive::GetOdomPose() const {
  return odom.GetPose();
}

frc::Pose2d SwerveDrive::GetPose() const {
  return poseEstimator.GetEstimatedPosition();
}

frc::Pose2d SwerveDrive::GetPredictedPose(units::second_t translationLookahead,
                                          units::second_t rotationLookahead) {
  frc::ChassisSpeeds currentVel = GetRobotRelativeSpeeds();
  return GetPose().TransformBy(
      frc::Transform2d{currentVel.vx * translationLookahead,
                       currentVel.vy * translationLookahead,
                       frc::Rotation2d{currentVel.omega * rotationLookahead}});
}

units::radian_t SwerveDrive::GetYawFromImu() {
  return yawLatencyComped;
}

std::array<units::radian_t, 4>
SwerveDrive::GetModuleDriveOutputShaftPositions() {
  return {modules[0].GetOutputShaftTurns(), modules[1].GetOutputShaftTurns(),
          modules[2].GetOutputShaftTurns(), modules[3].GetOutputShaftTurns()};
}

void SwerveDrive::SetModuleStates(
    const std::array<frc::SwerveModuleState, 4>& desiredStates, bool optimize,
    bool openLoop,
    const std::array<units::ampere_t, 4>& moduleTorqueCurrentsFF) {
  wpi::array<frc::SwerveModuleState, 4> finalState = desiredStates;
  frc::SwerveDriveKinematics<4>::DesaturateWheelSpeeds(
      &finalState, consts::swerve::physical::DRIVE_MAX_SPEED);
  int i = 0;
  for (auto& mod : modules) {
    finalState[i] = mod.GoToState(finalState[i], optimize, openLoop,
                                  moduleTorqueCurrentsFF[i]);
    i++;
  }
  desiredStatesPub.Set(finalState);
}

void SwerveDrive::AddVisionMeasurement(const frc::Pose2d& visionMeasurement,
                                       units::second_t timestamp,
                                       const Eigen::Vector3d& stdDevs) {
  // outside field, so we dont want to add this measurement to estimator,
  // because we know its wrong
  // TODO: Maybe add a small a buffer around the robots frame to add poses that
  // could be valid due to bumper compression
  if (math::IsPoseInsideField(visionMeasurement)) {
    wpi::array<double, 3> newStdDevs{stdDevs(0), stdDevs(1), stdDevs(2)};
    poseEstimator.AddVisionMeasurement(visionMeasurement, timestamp,
                                       newStdDevs);
  } else {
    // frc::DataLogManager::Log(
    //     "WARNING: Vision pose was outside of field! Not adding to estimator!");
  }
}

void SwerveDrive::UpdateSwerveOdom() {
  ctre::phoenix::StatusCode status =
      ctre::phoenix6::BaseStatusSignal::WaitForAll(
          2.0 / (1 / consts::SWERVE_ODOM_LOOP_PERIOD), allSignals);

  // if(!status.IsOK()) {
  //   frc::DataLogManager::Log(fmt::format("Error updating swerve odom! Error
  //   was: {}\n", status.GetName()));
  // }

  int i = 0;
  for (auto& mod : modules) {
    modulePositions[i] = mod.GetCurrentPosition(false);
    moduleStates[i] = mod.GetCurrentState();
    i++;
  }

  yawLatencyComped =
      ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
          imu.GetYaw(), imu.GetAngularVelocityZWorld());
  poseEstimator.Update(frc::Rotation2d{yawLatencyComped}, modulePositions);
  odom.Update(frc::Rotation2d{yawLatencyComped}, modulePositions);

  units::second_t now = frc::Timer::GetFPGATimestamp();
  odomUpdateRate = 1.0 / (now - lastOdomUpdateTime);
  lastOdomUpdateTime = now;
}

void SwerveDrive::UpdateNTEntries() {
  currentStatesPub.Set(moduleStates);
  currentPositionsPub.Set(modulePositions);
  odomPosePub.Set(GetOdomPose());
  //lookaheadPub.Set(GetPredictedPose(1_s, 1_s));
  estimatorPub.Set(GetPose());
  //isSlippingPub.Set(IsSlipping());
  odomUpdateRatePub.Set(odomUpdateRate.value());
  field.SetRobotPose(GetPose());
}

void SwerveDrive::SimulationPeriodic() {
  std::array<frc::SwerveModuleState, 4> simState;
  int i = 0;
  for (auto& swerveModule : modules) {
    simState[i] = swerveModule.UpdateSimulation(
        consts::LOOP_PERIOD, frc::RobotController::GetBatteryVoltage());
    i++;
  }

  simStatesPub.Set(simState);

  units::radians_per_second_t omega =
      consts::swerve::physical::KINEMATICS.ToChassisSpeeds(simState).omega;
  units::radian_t angleChange = omega * consts::LOOP_PERIOD;

  lastSimAngle = lastSimAngle + frc::Rotation2d{angleChange};
  imuSimState.SetRawYaw(lastSimAngle.Degrees());
}

units::ampere_t SwerveDrive::GetSimulatedCurrentDraw() const {
  units::ampere_t totalCurrent = 0_A;
  for (const auto& swerveModule : modules) {
    totalCurrent += swerveModule.GetSimulatedCurrentDraw();
  }
  return totalCurrent;
}

void SwerveDrive::ZeroYaw() {
  units::radian_t targetAngle = 0_rad;
  if(str::IsOnRed()) {
    targetAngle = 180_deg;
  }
  else {
    targetAngle = 0_deg;
  }
  imu.SetYaw(targetAngle);
}

void SwerveDrive::SetXModuleForces(
    const std::array<units::newton_t, 4>& xForce) {
  xModuleForce = xForce;
}

void SwerveDrive::SetYModuleForces(
    const std::array<units::newton_t, 4>& yForce) {
  yModuleForce = yForce;
}

void SwerveDrive::SetMk4iCharacterizationTorqueSteer(units::volt_t torqueAmps) {
  modules[0].SetSteerToTorque(torqueAmps);
}

void SwerveDrive::SetMk4nCharacterizationTorqueSteer(units::volt_t torqueAmps) {
  modules[2].SetSteerToTorque(torqueAmps);
}

void SwerveDrive::SetCharacterizationTorqueDrive(units::volt_t torqueAmps) {
  modules[0].SetDriveToTorque(torqueAmps);
  modules[1].SetDriveToTorque(torqueAmps);
  modules[2].SetDriveToTorque(torqueAmps);
  modules[3].SetDriveToTorque(torqueAmps);
}

void SwerveDrive::SetMk4iCharacterizationVoltageSteer(units::volt_t volts) {
  modules[0].SetSteerToVoltage(volts);
}

void SwerveDrive::SetMk4nCharacterizationVoltageSteer(units::volt_t volts) {
  modules[2].SetSteerToVoltage(volts);
}

void SwerveDrive::SetCharacterizationVoltageDrive(units::volt_t volts) {
  modules[0].SetDriveToVoltage(volts);
  modules[1].SetDriveToVoltage(volts);
  modules[2].SetDriveToVoltage(volts);
  modules[3].SetDriveToVoltage(volts);
}

void SwerveDrive::LogMk4iSteerTorque(frc::sysid::SysIdRoutineLog* log) {
  log->Motor("swerve-steer-mk4i")
      .voltage(units::volt_t{allSignals[5]->GetValueAsDouble()})
      .position(units::turn_t{allSignals[2]->GetValueAsDouble()})
      .velocity(units::turns_per_second_t{allSignals[3]->GetValueAsDouble()});
}

// This assumes the mk4n's are in the back of the robot
void SwerveDrive::LogMk4nSteerTorque(frc::sysid::SysIdRoutineLog* log) {
  log->Motor("swerve-steer-mk4n")
      .voltage(units::volt_t{allSignals[21]->GetValueAsDouble()})
      .position(units::turn_t{allSignals[18]->GetValueAsDouble()})
      .velocity(units::turns_per_second_t{allSignals[19]->GetValueAsDouble()});
}

void SwerveDrive::LogDriveTorque(frc::sysid::SysIdRoutineLog* log) {
  log->Motor("swerve-drive")
      .voltage(units::volt_t{allSignals[4]->GetValueAsDouble()})
      .position(units::turn_t{allSignals[0]->GetValueAsDouble()})
      .velocity(units::turns_per_second_t{allSignals[1]->GetValueAsDouble()});
}

void SwerveDrive::LogMk4iSteerVoltage(frc::sysid::SysIdRoutineLog* log) {
  log->Motor("swerve-steer-mk4i")
      .voltage(units::volt_t{allSignals[7]->GetValueAsDouble()})
      .position(units::turn_t{allSignals[2]->GetValueAsDouble()})
      .velocity(units::turns_per_second_t{allSignals[3]->GetValueAsDouble()});
}

// This assumes the mk4n's are in the back of the robot
void SwerveDrive::LogMk4nSteerVoltage(frc::sysid::SysIdRoutineLog* log) {
  log->Motor("swerve-steer-mk4n")
      .voltage(units::volt_t{allSignals[23]->GetValueAsDouble()})
      .position(units::turn_t{allSignals[18]->GetValueAsDouble()})
      .velocity(units::turns_per_second_t{allSignals[19]->GetValueAsDouble()});
}

void SwerveDrive::LogDriveVoltage(frc::sysid::SysIdRoutineLog* log) {
  log->Motor("swerve-drive")
      .voltage(units::volt_t{allSignals[6]->GetValueAsDouble()})
      .position(units::turn_t{allSignals[0]->GetValueAsDouble()})
      .velocity(units::turns_per_second_t{allSignals[1]->GetValueAsDouble()});
}

std::array<units::ampere_t, 4> SwerveDrive::ConvertModuleForcesToTorqueCurrent(
    const std::array<units::newton_t, 4>& xForce,
    const std::array<units::newton_t, 4>& yForce) {
  std::array<frc::SwerveModuleState, 4> forces;

  std::array<units::ampere_t, 4> retVal;
  for (int i = 0; i < 4; i++) {
    if(xForce[i] == 0_N && yForce[0] == 0_N) {
      break;
    }
    frc::Translation2d moduleForceFieldRef{units::meter_t{xForce[i].value()},
                                           units::meter_t{yForce[i].value()}};
    frc::Translation2d moduleForceRobotRef =
        moduleForceFieldRef.RotateBy(GetPose().Rotation());
    units::newton_meter_t totalTorqueAtMotor =
        (units::newton_t{moduleForceRobotRef.Norm().value()} *
         consts::swerve::physical::WHEEL_RADIUS) /
        consts::swerve::physical::DRIVE_GEARING;
    units::ampere_t expectedTorqueCurrent =
        totalTorqueAtMotor / consts::swerve::physical::DRIVE_MOTOR.Kt;
    retVal[i] = expectedTorqueCurrent;
    forces[i].angle = moduleForceRobotRef.Angle();
    forces[i].speed = units::feet_per_second_t{expectedTorqueCurrent.value()};
  }
  forcesPub.Set(forces);

  return retVal;
}

bool SwerveDrive::IsSlipping() {
  double slipCoeff = 1.5;
  frc::ChassisSpeeds robotRelSpeeds = GetRobotRelativeSpeeds();
  frc::ChassisSpeeds rotationComponent{0_mps, 0_mps, robotRelSpeeds.omega};
  frc::ChassisSpeeds translationComponent = robotRelSpeeds - rotationComponent;
  std::array<frc::SwerveModuleState, 4> transStates =
      consts::swerve::physical::KINEMATICS.ToSwerveModuleStates(
          translationComponent);
  auto maxIt = std::max_element(
      transStates.begin(), transStates.end(),
      [](const frc::SwerveModuleState& a, const frc::SwerveModuleState& b) {
        return a.speed < b.speed;
      });
  auto minIt = std::min_element(
      transStates.begin(), transStates.end(),
      [](const frc::SwerveModuleState& a, const frc::SwerveModuleState& b) {
        return a.speed < b.speed;
      });
  return (maxIt->speed / minIt->speed) > slipCoeff;
}

str::SwerveModuleSteerGains SwerveDrive::GetSteerGains() const {
  return modules[0].GetSteerGains();
}

void SwerveDrive::SetSteerGains(str::SwerveModuleSteerGains newGains) {
  for (int i = 0; i < 4; i++) {
    modules[i].SetSteerGains(newGains);
  }
}

str::SwerveModuleDriveGains SwerveDrive::GetDriveGains() const {
  return modules[0].GetDriveGains();
}

void SwerveDrive::SetDriveGains(str::SwerveModuleDriveGains newGains) {
  for (int i = 0; i < 4; i++) {
    modules[i].SetDriveGains(newGains);
  }
}
