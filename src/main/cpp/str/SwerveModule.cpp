#include "str/SwerveModule.h"
#include <ctre/phoenix/export.h>

using namespace str;

SwerveModule::SwerveModule(SwerveModuleConstants constants, SwerveModulePhysical physicalAttrib, SwerveModuleSteerGains steerGains, SwerveModuleDriveGains driveGains) 
: steerMotor(constants.steerId, "*"), 
  driveMotor(constants.driveId, "*"), 
  steerEncoder(constants.encoderId, "*"),
  moduleName(constants.moduleName),
  steerGains(steerGains),
  driveGains(driveGains),
  moduleSim(
    constants,
    physicalAttrib,
    driveMotor.GetSimState(),
    steerMotor.GetSimState(),
    steerEncoder.GetSimState()
  ),
  nt(nt::NetworkTableInstance::GetDefault().GetTable(moduleName + "_SwerveModule")),
  desiredStateTopic(nt->GetStructTopic<frc::SwerveModuleState>("DesiredState")),
  desiredStatePub(desiredStateTopic.Publish())
{
  ConfigureSteerMotor(constants.invertSteer, physicalAttrib.steerGearing, physicalAttrib.supplySideLimit);
  ConfigureDriveMotor(constants.invertDrive, physicalAttrib.supplySideLimit, physicalAttrib.slipCurrent);
  ConfigureSteerEncoder(constants.steerEncoderOffset);
}

void SwerveModule::GoToState(frc::SwerveModuleState desiredState) {
  desiredStatePub.Set(desiredState);
}

frc::SwerveModuleState SwerveModule::GetCurrentState() const {

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

bool SwerveModule::ConfigureDriveMotor(bool invertDrive, units::ampere_t supplyCurrentLimit, units::ampere_t slipCurrentLimit) {
  ctre::phoenix6::configs::TalonFXConfiguration driveConfig{};
  ctre::phoenix6::configs::Slot0Configs driveSlotConfig{};

  //Gains are dependent on control mode. By default, use TorqueCurrentFOC
  driveSlotConfig.kV = driveGains.kV.value();
  driveSlotConfig.kA = driveGains.kA.value();
  driveSlotConfig.kS = driveGains.kS.value();
  driveSlotConfig.kP = driveGains.kP.value();
  driveSlotConfig.kI = driveGains.kI.value();
  driveSlotConfig.kD = driveGains.kD.value();
  driveConfig.Slot0 = driveSlotConfig;

  driveConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
  driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = slipCurrentLimit.value();
  driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -slipCurrentLimit.value();
  driveConfig.CurrentLimits.StatorCurrentLimit = slipCurrentLimit.value();
  driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
  driveConfig.MotorOutput.Inverted =
      invertDrive
          ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
          : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;

  //Supply side limiting only effects non torque modes. We should set these anyway in case we want to control in a different mode
  driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  driveConfig.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit.value();

  ctre::phoenix::StatusCode configResult = driveMotor.GetConfigurator().Apply(driveConfig);

  fmt::print("Configured drive motor on module {}. Result was: {}\n", moduleName, configResult.GetName());

  return configResult.IsOK();
}

bool SwerveModule::ConfigureSteerEncoder(double encoderOffset) {
  ctre::phoenix6::configs::CANcoderConfiguration encoderConfig{};
  encoderConfig.MagnetSensor.MagnetOffset = encoderOffset;
  ctre::phoenix::StatusCode configResult = steerEncoder.GetConfigurator().Apply(encoderConfig);

  fmt::print("Configured steer encoder on module {}. Result was: {}\n", moduleName, configResult.GetName());
  
  return configResult.IsOK();
}