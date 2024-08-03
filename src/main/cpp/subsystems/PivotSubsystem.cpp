// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "subsystems/PivotSubsystem.h"

PivotSubsystem::PivotSubsystem() {
  ConfigurePivotMotors();
  ConfigurePivotEncoder();
}

// This method will be called once per scheduler run
void PivotSubsystem::Periodic() {}

bool PivotSubsystem::ConfigurePivotMotors() {
  ctre::phoenix6::configs::TalonFXConfiguration pivotConfig{};
  ctre::phoenix6::configs::Slot0Configs pivotSlotConfig{};

  // Gains are dependent on control mode. By default, use TorqueCurrentFOC with
  // Motion Magic Expo.
  pivotSlotConfig.kA = consts::pivot::gains::kA.value();
  pivotSlotConfig.kV = consts::pivot::gains::kV.value();
  pivotSlotConfig.kS = consts::pivot::gains::kS.value();
  pivotSlotConfig.kP = consts::pivot::gains::kP.value();
  pivotSlotConfig.kI = consts::pivot::gains::kI.value();
  pivotSlotConfig.kD = consts::pivot::gains::kD.value();
  pivotSlotConfig.GravityType = ctre::phoenix6::signals::GravityTypeValue::Arm_Cosine;
  pivotSlotConfig.kG = consts::pivot::gains::kG.value();
  pivotConfig.Slot0 = pivotSlotConfig;

  pivotConfig.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Brake;
  pivotConfig.Feedback.FeedbackRemoteSensorID = pivotEncoder.GetDeviceID();
  pivotConfig.Feedback.FeedbackSensorSource =
      ctre::phoenix6::signals::FeedbackSensorSourceValue::FusedCANcoder;
  pivotConfig.Feedback.RotorToSensorRatio =
      consts::pivot::physical::PIVOT_GEARING;
  pivotConfig.MotorOutput.Inverted = false;
  pivotConfig.ClosedLoopGeneral.ContinuousWrap = false;

  pivotConfig.MotionMagic.MotionMagicCruiseVelocity =
      consts::pivot::gains::motionMagicCruiseVel.value();
  // Motion magic expo kv and ka are always in terms of volts, even if we are
  // controlling current.
  pivotConfig.MotionMagic.MotionMagicExpo_kV =
      consts::pivot::gains::motionMagicExpoKv.value();
  pivotConfig.MotionMagic.MotionMagicExpo_kA =
      consts::pivot::gains::motionMagicExpoKa.value();

  pivotConfig.TorqueCurrent.PeakForwardTorqueCurrent =
      consts::pivot::current_limits::STEER_TORQUE_CURRENT_LIMIT.value();
  pivotConfig.TorqueCurrent.PeakReverseTorqueCurrent =
      -consts::pivot::current_limits::STEER_TORQUE_CURRENT_LIMIT.value();

  // Supply side limiting only effects non torque modes. We should set these
  // anyway in case we want to control in a different mode
  pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  pivotConfig.CurrentLimits.SupplyCurrentLimit =
      consts::pivot::current_limits::SUPPLY_CURRENT_LIMIT.value();

  ctre::phoenix::StatusCode configResult =
      pivotLeft.GetConfigurator().Apply(pivotConfig);

  pivotRight.SetControl(
      ctre::phoenix6::controls::Follower{pivotLeft.GetDeviceID(), true});

  fmt::print("Configured pivot motors. Result was: {}\n",
             configResult.GetName());

  return configResult.IsOK();
}

bool PivotSubsystem::ConfigurePivotEncoder() {
  ctre::phoenix6::configs::CANcoderConfiguration encoderConfig{};
  encoderConfig.MagnetSensor.MagnetOffset =
      consts::pivot::physical::ENCODER_OFFSET.value();
  encoderConfig.MagnetSensor.AbsoluteSensorRange =
      ctre::phoenix6::signals::AbsoluteSensorRangeValue::Unsigned_0To1;
  encoderConfig.MagnetSensor.SensorDirection =
      ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
  ctre::phoenix::StatusCode configResult =
      pivotEncoder.GetConfigurator().Apply(encoderConfig);

  fmt::print("Configured pivot encoder. Result was: {}\n",
             configResult.GetName());

  return configResult.IsOK();
}

void PivotSubsystem::ConfigureControlSignals() {}
