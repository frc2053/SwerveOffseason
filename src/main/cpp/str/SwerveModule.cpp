// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "str/SwerveModule.h"

#include <frc/DataLogManager.h>
#include <frc/MathUtil.h>

#include <string>

using namespace str;

SwerveModule::SwerveModule(SwerveModuleConstants constants,
                           SwerveModulePhysical physicalAttrib,
                           SwerveModuleSteerGains steerGains,
                           SwerveModuleDriveGains driveGains)
    : steerMotor(constants.steerId, "*"),
      driveMotor(constants.driveId, "*"),
      steerEncoder(constants.encoderId, "*"),
      moduleName(constants.moduleName),
      steerGains(steerGains),
      driveGains(driveGains),
      couplingRatio(physicalAttrib.couplingRatio),
      driveGearing(physicalAttrib.driveGearing),
      wheelRadius(physicalAttrib.wheelRadius),
      maxLinearSpeed(
          ((physicalAttrib.driveMotor.freeSpeed / 1_rad) / driveGearing) *
          wheelRadius),
      moduleSim(constants, physicalAttrib, driveMotor.GetSimState(),
                steerMotor.GetSimState(), steerEncoder.GetSimState()) {
  if (!ConfigureSteerMotor(constants.invertSteer, physicalAttrib.steerGearing,
                           physicalAttrib.steerTorqueCurrentLimit,
                           physicalAttrib.steerTorqueCurrentLimit)) {
    frc::DataLogManager::Log("ERROR: Failed to configure steer motor!");
  }
  if (!ConfigureDriveMotor(constants.invertDrive,
                           physicalAttrib.driveSupplySideLimit,
                           physicalAttrib.slipCurrent)) {
    frc::DataLogManager::Log("ERROR: Failed to configure drive motor!");
  }
  if (!ConfigureSteerEncoder(constants.steerEncoderOffset)) {
    frc::DataLogManager::Log("ERROR: Failed to configure steer encoder!");
  }
  ConfigureControlSignals();
}

frc::SwerveModuleState SwerveModule::GoToState(
    frc::SwerveModuleState desiredState, bool optimize, bool openLoopDrive,
    units::ampere_t arbff) {
  frc::SwerveModuleState currentState = GetCurrentState();
  if (optimize) {
    desiredState.Optimize(currentState.angle);
  }

  desiredState.CosineScale(currentState.angle);

  steerMotor.SetControl(
      steerAngleSetter.WithPosition(desiredState.angle.Radians()));

  units::radians_per_second_t motorSpeed =
      ConvertWheelVelToMotorVel(ConvertLinearVelToWheelVel(desiredState.speed));

  // Reverse the modules expected backout because of coupling
  units::radians_per_second_t driveBackout =
      steerVelocitySig.GetValue() * couplingRatio;
  motorSpeed += driveBackout;

  if (openLoopDrive) {
    driveMotor.SetControl(driveVoltageSetter.WithOutput(
        (motorSpeed / ConvertWheelVelToMotorVel(
                          ConvertLinearVelToWheelVel(maxLinearSpeed))) *
        12_V));
  } else {
    driveMotor.SetControl(
        driveVelocitySetter.WithVelocity(motorSpeed)
            .WithFeedForward(units::math::copysign(arbff, motorSpeed)));
  }

  // Just for logging
  desiredState.speed =
      ConvertWheelVelToLinearVel(ConvertDriveMotorVelToWheelVel(motorSpeed));

  return desiredState;
}

frc::SwerveModulePosition SwerveModule::GetCurrentPosition(bool refresh) {
  if (refresh) {
    ctre::phoenix::StatusCode moduleSignalStatus =
        ctre::phoenix6::BaseStatusSignal::WaitForAll(
            0_s, drivePositionSig, driveVelocitySig, driveVoltageSig,
            steerPositionSig, steerVelocitySig, steerVoltageSig);

    if (!moduleSignalStatus.IsOK()) {
      frc::DataLogManager::Log(fmt::format(
          "Error refreshing {} module signal in GetCurrentPosition()! "
          "Error was: {}\n",
          moduleName, moduleSignalStatus.GetName()));
    }
  }

  units::radian_t latencyCompSteerPos =
      ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
          steerPositionSig, steerVelocitySig);
  units::radian_t latencyCompDrivePos =
      ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
          drivePositionSig, driveVelocitySig);

  // The drive and steer run on the same gear train, rotating the module will
  // slightly move the drive wheel. We account for this here.
  latencyCompDrivePos -= latencyCompSteerPos * couplingRatio;

  frc::SwerveModulePosition position{
      ConvertWheelRotationsToWheelDistance(
          ConvertDriveMotorRotationsToWheelRotations(latencyCompDrivePos)),
      frc::Rotation2d{latencyCompSteerPos}};

  return position;
}

frc::SwerveModuleState SwerveModule::GetCurrentState() {
  frc::SwerveModuleState currentState{
      ConvertWheelVelToLinearVel(
          ConvertDriveMotorVelToWheelVel(driveVelocitySig.GetValue())),
      frc::Rotation2d{steerPositionSig.GetValue()}};

  return currentState;
}

units::radian_t SwerveModule::GetOutputShaftTurns() {
  ctre::phoenix::StatusCode moduleSignalStatus =
      ctre::phoenix6::BaseStatusSignal::WaitForAll(0_s, drivePositionSig,
                                                   driveVelocitySig);

  if (!moduleSignalStatus.IsOK()) {
    frc::DataLogManager::Log(fmt::format(
        "Error refreshing {} module signal in GetDriveMotorTurns()! "
        "Error was: {}\n",
        moduleName, moduleSignalStatus.GetName()));
  }

  units::radian_t latencyCompDrivePos =
      ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
          drivePositionSig, driveVelocitySig);

  return ConvertDriveMotorRotationsToWheelRotations(latencyCompDrivePos);
}

frc::SwerveModuleState SwerveModule::UpdateSimulation(
    units::second_t deltaTime, units::volt_t supplyVoltage) {
  return moduleSim.Update(deltaTime, supplyVoltage);
}

std::array<ctre::phoenix6::BaseStatusSignal*, 8> SwerveModule::GetSignals() {
  return {&drivePositionSig, &driveVelocitySig,      &steerPositionSig,
          &steerVelocitySig, &driveTorqueCurrentSig, &steerTorqueCurrentSig,
          &driveVoltageSig,  &steerVoltageSig};
}

bool SwerveModule::ConfigureSteerMotor(bool invertSteer,
                                       units::scalar_t steerGearing,
                                       units::ampere_t supplyCurrentLimit,
                                       units::ampere_t torqueCurrentLimit) {
  ctre::phoenix6::configs::TalonFXConfiguration steerConfig{};
  ctre::phoenix6::configs::Slot0Configs steerSlotConfig{};

  // Gains are dependent on control mode. By default, use TorqueCurrentFOC with
  // Motion Magic Expo.
  steerSlotConfig.kA = steerGains.kA.value();
  steerSlotConfig.kV = steerGains.kV.value();
  steerSlotConfig.kS = steerGains.kS.value();
  steerSlotConfig.kP = steerGains.kP.value();
  steerSlotConfig.kI = steerGains.kI.value();
  steerSlotConfig.kD = steerGains.kD.value();
  steerConfig.Slot0 = steerSlotConfig;

  steerConfig.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Brake;
  steerConfig.Feedback.FeedbackRemoteSensorID = steerEncoder.GetDeviceID();
  steerConfig.Feedback.FeedbackSensorSource =
      ctre::phoenix6::signals::FeedbackSensorSourceValue::FusedCANcoder;
  steerConfig.Feedback.RotorToSensorRatio = steerGearing;
  steerConfig.MotorOutput.Inverted =
      invertSteer
          ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
          : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  steerConfig.ClosedLoopGeneral.ContinuousWrap = true;

  steerConfig.MotionMagic.MotionMagicCruiseVelocity =
      steerGains.motionMagicCruiseVel.value();
  // Motion magic expo kv and ka are always in terms of volts, even if we are
  // controlling current.
  steerConfig.MotionMagic.MotionMagicExpo_kV =
      steerGains.motionMagicExpoKv.value();
  steerConfig.MotionMagic.MotionMagicExpo_kA =
      steerGains.motionMagicExpoKa.value();

  steerConfig.TorqueCurrent.PeakForwardTorqueCurrent =
      torqueCurrentLimit.value();
  steerConfig.TorqueCurrent.PeakReverseTorqueCurrent =
      -torqueCurrentLimit.value();

  // Supply side limiting only effects non torque modes. We should set these
  // anyway in case we want to control in a different mode
  steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  steerConfig.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit.value();

  ctre::phoenix::StatusCode configResult =
      steerMotor.GetConfigurator().Apply(steerConfig);

  frc::DataLogManager::Log(
      fmt::format("Configured steer motor on module {}. Result was: {}\n",
                  moduleName, configResult.GetName()));

  return configResult.IsOK();
}

bool SwerveModule::ConfigureDriveMotor(bool invertDrive,
                                       units::ampere_t supplyCurrentLimit,
                                       units::ampere_t slipCurrentLimit) {
  ctre::phoenix6::configs::TalonFXConfiguration driveConfig{};
  ctre::phoenix6::configs::Slot0Configs driveSlotConfig{};

  // Gains are dependent on control mode. By default, use TorqueCurrentFOC
  driveSlotConfig.kV = driveGains.kV.value();
  driveSlotConfig.kA = driveGains.kA.value();
  driveSlotConfig.kS = driveGains.kS.value();
  driveSlotConfig.kP = driveGains.kP.value();
  driveSlotConfig.kI = driveGains.kI.value();
  driveSlotConfig.kD = driveGains.kD.value();
  driveConfig.Slot0 = driveSlotConfig;

  driveConfig.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Brake;
  driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = slipCurrentLimit.value();
  driveConfig.TorqueCurrent.PeakReverseTorqueCurrent =
      -slipCurrentLimit.value();
  driveConfig.CurrentLimits.StatorCurrentLimit = slipCurrentLimit.value();
  driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
  driveConfig.MotorOutput.Inverted =
      invertDrive
          ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
          : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;

  // Supply side limiting only effects non torque modes. We should set these
  // anyway in case we want to control in a different mode
  driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  driveConfig.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit.value();

  ctre::phoenix::StatusCode configResult =
      driveMotor.GetConfigurator().Apply(driveConfig);

  frc::DataLogManager::Log(
      fmt::format("Configured drive motor on module {}. Result was: {}\n",
                  moduleName, configResult.GetName()));

  return configResult.IsOK();
}

bool SwerveModule::ConfigureSteerEncoder(units::turn_t encoderOffset) {
  ctre::phoenix6::configs::CANcoderConfiguration encoderConfig{};
  encoderConfig.MagnetSensor.MagnetOffset = encoderOffset.value();
  encoderConfig.MagnetSensor.AbsoluteSensorRange =
      ctre::phoenix6::signals::AbsoluteSensorRangeValue::Signed_PlusMinusHalf;
  encoderConfig.MagnetSensor.SensorDirection =
      ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
  ctre::phoenix::StatusCode configResult =
      steerEncoder.GetConfigurator().Apply(encoderConfig);

  frc::DataLogManager::Log(
      fmt::format("Configured steer encoder on module {}. Result was: {}\n",
                  moduleName, configResult.GetName()));

  return configResult.IsOK();
}

void SwerveModule::ConfigureControlSignals() {
  steerAngleSetter.UpdateFreqHz = 0_Hz;
  driveVelocitySetter.UpdateFreqHz = 0_Hz;
  steerTorqueSetter.UpdateFreqHz = 0_Hz;
  driveTorqueSetter.UpdateFreqHz = 0_Hz;
  steerVoltageSetter.UpdateFreqHz = 0_Hz;
  driveVoltageSetter.UpdateFreqHz = 0_Hz;
  // Velocity Torque current neutral should always be coast, as neutral
  // corresponds to 0-current or maintain velocity, not 0-velocity
  driveVelocitySetter.OverrideCoastDurNeutral = true;
  steerTorqueSetter.OverrideCoastDurNeutral = true;
  driveTorqueSetter.OverrideCoastDurNeutral = true;
}

bool SwerveModule::OptimizeBusSignals() {
  ctre::phoenix::StatusCode optimizeDriveResult =
      driveMotor.OptimizeBusUtilization();
  if (optimizeDriveResult.IsOK()) {
    frc::DataLogManager::Log(
        fmt::format("Optimized bus signals for {} drive motor\n", moduleName));
  }
  ctre::phoenix::StatusCode optimizeSteerResult =
      steerMotor.OptimizeBusUtilization();
  if (optimizeSteerResult.IsOK()) {
    frc::DataLogManager::Log(
        fmt::format("Optimized bus signals for {} steer motor\n", moduleName));
  }
  return optimizeDriveResult.IsOK() && optimizeSteerResult.IsOK();
}

std::string SwerveModule::GetName() const {
  return moduleName;
}

units::ampere_t SwerveModule::GetSimulatedCurrentDraw() const {
  return moduleSim.GetDriveCurrentDraw() + moduleSim.GetSteerCurrentDraw();
}

void SwerveModule::SetSteerGains(str::SwerveModuleSteerGains newGains) {
  steerGains = newGains;
  ctre::phoenix6::configs::Slot0Configs steerSlotConfig{};
  steerSlotConfig.kV = steerGains.kV.value();
  steerSlotConfig.kA = steerGains.kA.value();
  steerSlotConfig.kS = steerGains.kS.value();
  steerSlotConfig.kP = steerGains.kP.value();
  steerSlotConfig.kI = steerGains.kI.value();
  steerSlotConfig.kD = steerGains.kD.value();

  ctre::phoenix6::configs::MotionMagicConfigs steerMMConfig{};

  steerMMConfig.MotionMagicCruiseVelocity =
      steerGains.motionMagicCruiseVel.value();
  steerMMConfig.MotionMagicExpo_kV = steerGains.motionMagicExpoKv.value();
  steerMMConfig.MotionMagicExpo_kA = steerGains.motionMagicExpoKa.value();

  ctre::phoenix::StatusCode statusGains =
      steerMotor.GetConfigurator().Apply(steerSlotConfig);
  if (!statusGains.IsOK()) {
    frc::DataLogManager::Log(
        fmt::format("Swerve Steer Motor was unable to set new gains! "
                    "Error: {}, More Info: {}",
                    statusGains.GetName(), statusGains.GetDescription()));
  }

  ctre::phoenix::StatusCode statusMM =
      steerMotor.GetConfigurator().Apply(steerMMConfig);
  if (!statusMM.IsOK()) {
    frc::DataLogManager::Log(fmt::format(
        "Swerve Steer Motor was unable to set new motion magic config! "
        "Error: {}, More Info: {}",
        statusMM.GetName(), statusMM.GetDescription()));
  }
}

str::SwerveModuleSteerGains SwerveModule::GetSteerGains() const {
  return steerGains;
}

str::SwerveModuleDriveGains SwerveModule::GetDriveGains() const {
  return driveGains;
}

void SwerveModule::SetDriveGains(str::SwerveModuleDriveGains newGains) {
  driveGains = newGains;
  ctre::phoenix6::configs::Slot0Configs driveSlotConfig{};
  driveSlotConfig.kV = driveGains.kV.value();
  driveSlotConfig.kA = driveGains.kA.value();
  driveSlotConfig.kS = driveGains.kS.value();
  driveSlotConfig.kP = driveGains.kP.value();
  driveSlotConfig.kI = driveGains.kI.value();
  driveSlotConfig.kD = driveGains.kD.value();
  ctre::phoenix::StatusCode status =
      driveMotor.GetConfigurator().Apply(driveSlotConfig);
  if (!status.IsOK()) {
    frc::DataLogManager::Log(
        fmt::format("Swerve Drive Motor was unable to set new gains! "
                    "Error: {}, More Info: {}",
                    status.GetName(), status.GetDescription()));
  }
}

void SwerveModule::SetSteerToTorque(units::volt_t voltsToSend) {
  steerMotor.SetControl(
      steerTorqueSetter.WithOutput(units::ampere_t{voltsToSend.value()}));
}

void SwerveModule::SetDriveToTorque(units::volt_t voltsToSend) {
  driveMotor.SetControl(
      driveTorqueSetter.WithOutput(units::ampere_t{voltsToSend.value()}));
}

void SwerveModule::SetSteerToVoltage(units::volt_t voltsToSend) {
  steerMotor.SetControl(steerVoltageSetter.WithOutput(voltsToSend));
}

void SwerveModule::SetDriveToVoltage(units::volt_t voltsToSend) {
  driveMotor.SetControl(driveVoltageSetter.WithOutput(voltsToSend));
}

units::radian_t SwerveModule::ConvertDriveMotorRotationsToWheelRotations(
    units::radian_t motorRotations) const {
  return motorRotations / driveGearing;
}

units::radians_per_second_t SwerveModule::ConvertDriveMotorVelToWheelVel(
    units::radians_per_second_t motorVel) const {
  return motorVel / driveGearing;
}

units::meter_t SwerveModule::ConvertWheelRotationsToWheelDistance(
    units::radian_t wheelRotations) const {
  return (wheelRotations / 1_rad) * wheelRadius;
}

units::meters_per_second_t SwerveModule::ConvertWheelVelToLinearVel(
    units::radians_per_second_t wheelVel) const {
  return (wheelVel / 1_rad) * wheelRadius;
}

units::radians_per_second_t SwerveModule::ConvertLinearVelToWheelVel(
    units::meters_per_second_t linVel) const {
  return (linVel / wheelRadius) * 1_rad;
}

units::radians_per_second_t SwerveModule::ConvertWheelVelToMotorVel(
    units::radians_per_second_t wheelVel) const {
  return wheelVel * driveGearing;
}
