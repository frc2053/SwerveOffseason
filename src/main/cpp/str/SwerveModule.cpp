#include "str/SwerveModule.h"
#include <ctre/phoenix/export.h>

using namespace str;

SwerveModule::SwerveModule(SwerveModuleConstants constants, SwerveModulePhysical physicalAttrib, SwerveModuleSteerGains steerGains) 
: steerMotor(constants.steerId, "*"), 
  driveMotor(constants.driveId, "*"), 
  steerEncoder(constants.encoderId, "*"),
  moduleName(constants.moduleName),
  steerGains(steerGains)
{
  ConfigureSteerMotor(constants.invertSteer, physicalAttrib.steerGearing, physicalAttrib.supplySideLimit);
  ConfigureDriveMotor(constants.invertDrive);
  ConfigureSteerEncoder(constants.steerEncoderOffset);
}

bool SwerveModule::ConfigureSteerMotor(bool invertSteer, units::scalar_t steerGearing, units::ampere_t supplyCurrentLimit) {
  ctre::phoenix6::configs::TalonFXConfiguration steerConfig{};
  ctre::phoenix6::configs::Slot0Configs steerSlotConfig{};

  //Gains are dependent on control mode. By default, use TorqueCurrentFOC with Motion Magic Expo.
  steerSlotConfig.kA = steerGains.kA.value();
  steerSlotConfig.kV = steerGains.kV.value();
  steerSlotConfig.kS = steerGains.kS.value();
  steerSlotConfig.kP = steerGains.kP.value();
  steerSlotConfig.kI = steerGains.kI.value();
  steerSlotConfig.kD = steerGains.kD.value();
  steerConfig.Slot0 = steerSlotConfig;

  steerConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
  steerConfig.Feedback.FeedbackRemoteSensorID = steerEncoder.GetDeviceID();
  steerConfig.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::FusedCANcoder;
  steerConfig.Feedback.RotorToSensorRatio = steerGearing;
  steerConfig.MotorOutput.Inverted =
      invertSteer
          ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
          : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  steerConfig.ClosedLoopGeneral.ContinuousWrap = true;

  steerConfig.MotionMagic.MotionMagicCruiseVelocity = steerGains.motionMagicCruiseVel.value();
  //Motion magic expo kv and ka are always in terms of volts, even if we are controlling current.
  steerConfig.MotionMagic.MotionMagicExpo_kV = steerGains.motionMagicExpoKv.value();
  steerConfig.MotionMagic.MotionMagicExpo_kA = steerGains.motionMagicExpoKa.value();

  //Supply side limiting only effects non torque modes. We should set these anyway in case we want to control in a different mode
  steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  steerConfig.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit.value();

  ctre::phoenix::StatusCode configResult = steerMotor.GetConfigurator().Apply(steerConfig);

  fmt::print("Configured steer motor on module {}. Result was: {}\n", moduleName, configResult.GetName());

  return configResult.IsOK();
}

bool SwerveModule::ConfigureDriveMotor(bool invertDrive) {
  return false;
}

bool SwerveModule::ConfigureSteerEncoder(double encoderOffset) {
  ctre::phoenix6::configs::CANcoderConfiguration encoderConfig{};
  encoderConfig.MagnetSensor.MagnetOffset = encoderOffset;
  ctre::phoenix::StatusCode configResult = steerEncoder.GetConfigurator().Apply(encoderConfig);

  fmt::print("Configured steer encoder on module {}. Result was: {}\n", moduleName, configResult.GetName());
  
  return configResult.IsOK();
}