// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "subsystems/ShooterSubsystem.h"
#include "constants/Constants.h"

ShooterSubsystem::ShooterSubsystem() {
  ConfigureShooterMotors(consts::shooter::physical::BOTTOM_INVERT,
                         consts::shooter::physical::TOP_INVERT,
                         consts::shooter::physical::SHOOTER_RATIO,
                         consts::shooter::current_limits::SUPPLY_CURRENT_LIMIT,
                         consts::shooter::current_limits::STATOR_CURRENT_LIMIT);
  ConfigureMotorSignals();
}

// This method will be called once per scheduler run
void ShooterSubsystem::Periodic() {
    ctre::phoenix::StatusCode shooterWaitResult = ctre::phoenix6::BaseStatusSignal::RefreshAll({&topMotorVelSig, &topMotorVoltageSig, &bottomMotorVelSig, &bottomMotorVoltageSig});

    if(!shooterWaitResult.IsOK()) {
        fmt::print("Error grabbing shooter signals! Details: {}\n", shooterWaitResult.GetName());
    }

    currentBottomWheelVelocity = bottomMotorVelSig.GetValue();
    currentTopWheelVelocity = topMotorVelSig.GetValue();

    topWheelMotor.SetControl(topMotorSetter.WithOutput(10_V));
    bottomWheelMotor.SetControl(bottomMotorSetter.WithOutput(10_V));

    bottomWheelVelocityPub.Set(currentBottomWheelVelocity.convert<units::revolutions_per_minute>().value());
    topWheelVelocityPub.Set(currentTopWheelVelocity.convert<units::revolutions_per_minute>().value());
}

void ShooterSubsystem::SimulationPeriodic() {
    topMotorSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
    bottomMotorSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

    topFlywheelSim.SetInputVoltage(topMotorSim.GetMotorVoltage());
    bottomFlywheelSim.SetInputVoltage(bottomMotorSim.GetMotorVoltage());

    topFlywheelSim.Update(consts::LOOP_PERIOD);
    bottomFlywheelSim.Update(consts::LOOP_PERIOD);

    topMotorSim.SetRotorVelocity(topFlywheelSim.GetAngularVelocity());
    bottomMotorSim.SetRotorVelocity(bottomFlywheelSim.GetAngularVelocity());
}

bool ShooterSubsystem::IsUpToSpeed() {
    return (units::math::abs(topWheelVelocitySetpoint - currentTopWheelVelocity) < consts::shooter::gains::VEL_TOLERANCE) &&
    (units::math::abs(bottomWheelVelocitySetpoint - currentBottomWheelVelocity) < consts::shooter::gains::VEL_TOLERANCE);
}

bool ShooterSubsystem::ConfigureShooterMotors(
    bool invertBottom, bool invertTop, units::scalar_t shooterGearing,
    units::ampere_t supplyCurrentLimit, units::ampere_t statorCurrentLimit) {

  ctre::phoenix6::configs::TalonFXConfiguration shooterConfig{};
  ctre::phoenix6::configs::Slot0Configs shooterSlotConfig{};

  shooterSlotConfig.kV = consts::shooter::gains::SHOOTER_KV.value();
  shooterSlotConfig.kA = consts::shooter::gains::SHOOTER_KA.value();
  shooterSlotConfig.kS = consts::shooter::gains::SHOOTER_KS.value();
  shooterSlotConfig.kP = consts::shooter::gains::SHOOTER_KP.value();
  shooterSlotConfig.kI = consts::shooter::gains::SHOOTER_KI.value();
  shooterSlotConfig.kD = consts::shooter::gains::SHOOTER_KD.value();
  shooterConfig.Slot0 = shooterSlotConfig;

  shooterConfig.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Coast;
  shooterConfig.CurrentLimits.StatorCurrentLimit = statorCurrentLimit.value();
  shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;

  shooterConfig.MotorOutput.Inverted =
      invertTop
          ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
          : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;

  shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  shooterConfig.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit.value();

  ctre::phoenix::StatusCode topConfigResult =
      topWheelMotor.GetConfigurator().Apply(shooterConfig);

  fmt::print("Configured top shooter motor. Result was: {}\n",
             topConfigResult.GetName());

  shooterConfig.MotorOutput.Inverted =
      invertBottom
          ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
          : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;

  ctre::phoenix::StatusCode bottomConfigResult =
      bottomWheelMotor.GetConfigurator().Apply(shooterConfig);

  fmt::print("Configured bottom shooter motor. Result was: {}\n",
             bottomConfigResult.GetName());

  return bottomConfigResult.IsOK() && topConfigResult.IsOK();
}

bool ShooterSubsystem::ConfigureMotorSignals() {
  topMotorSetter.UpdateFreqHz = 0_Hz;
  bottomMotorSetter.UpdateFreqHz = 0_Hz;

  //Double the rio update rate? Not sure what is optimal here
  units::hertz_t updateRate = 1.0 / (consts::LOOP_PERIOD * 2.0);

  topMotorVelSig.SetUpdateFrequency(updateRate);
  topMotorVoltageSig.SetUpdateFrequency(updateRate);
  bottomMotorVelSig.SetUpdateFrequency(updateRate);
  bottomMotorVoltageSig.SetUpdateFrequency(updateRate);

  ctre::phoenix::StatusCode optimizeTopMotor =
      topWheelMotor.OptimizeBusUtilization();
  if (optimizeTopMotor.IsOK()) {
    fmt::print("Optimized bus signals for top shooter motor\n");
  }

  ctre::phoenix::StatusCode optimizeBottomMotor =
      bottomWheelMotor.OptimizeBusUtilization();
  if (optimizeBottomMotor.IsOK()) {
    fmt::print("Optimized bus signals for bottom shooter motor\n");
  }

  return optimizeTopMotor.IsOK() && optimizeBottomMotor.IsOK();
}