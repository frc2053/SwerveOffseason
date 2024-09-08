// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/FeederSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DataLogManager.h>
#include "constants/Constants.h"

FeederSubsystem::FeederSubsystem() {
    SetName("FeederSubsystem");
    ConfigureFeederMotor(consts::feeder::physical::INVERT_MOTOR, consts::feeder::physical::FEEDER_RATIO, consts::feeder::current_limits::SUPPLY_CURRENT_LIMIT, consts::feeder::current_limits::STATOR_CURRENT_LIMIT);
    ConfigureMotorSignals();
    frc::SmartDashboard::PutData(this);
}

// This method will be called once per scheduler run
void FeederSubsystem::Periodic() {
    UpdateNTEntries();
}

void FeederSubsystem::SimulationPeriodic() {
    feederMotorSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
    feederSim.SetInputVoltage(feederMotorSim.GetMotorVoltage());
    feederSim.Update(consts::LOOP_PERIOD);
    feederMotorSim.SetRotorVelocity(feederSim.GetAngularVelocity() * consts::feeder::physical::FEEDER_RATIO);
}

void FeederSubsystem::UpdateNTEntries() {
  feederWheelMotorVoltagePub.Set(currentFeederWheelVoltage.value());
  feederWheelMotorVoltageSetpointPub.Set(feederWheelVoltageSetpoint.value());
  hasNotePub.Set(hasNote);
}

bool FeederSubsystem::ConfigureFeederMotor(bool invert,
                            units::scalar_t intakeGearing,
                            units::ampere_t supplyCurrentLimit,
                            units::ampere_t statorCurrentLimit) {

  ctre::phoenix6::configs::TalonFXConfiguration feederConfig{};

  feederConfig.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Brake;
  feederConfig.CurrentLimits.StatorCurrentLimit = statorCurrentLimit.value();
  feederConfig.CurrentLimits.StatorCurrentLimitEnable = true;

  feederConfig.MotorOutput.Inverted =
      invert
          ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
          : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;

  feederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  feederConfig.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit.value();

  ctre::phoenix::StatusCode feederConfigResult =
      feederMotor.GetConfigurator().Apply(feederConfig);

  frc::DataLogManager::Log(fmt::format("Configured feeder motor. Result was: {}\n",
             feederConfigResult.GetName()));

  return feederConfigResult.IsOK();
}


bool FeederSubsystem::ConfigureMotorSignals() {
  feederMotorVoltageSetter.UpdateFreqHz = 0_Hz;

  //Double the rio update rate? Not sure what is optimal here
  units::hertz_t updateRate = 1.0 / (consts::LOOP_PERIOD * 2.0);

  feederMotorVoltageSig.SetUpdateFrequency(updateRate);
  feederMotorTorqueCurrentSig.SetUpdateFrequency(updateRate);

  ctre::phoenix::StatusCode optimizeFeederMotor =
      feederMotor.OptimizeBusUtilization();
  if (optimizeFeederMotor.IsOK()) {
    frc::DataLogManager::Log("Optimized bus signals for feeder motor\n");
  }

  return optimizeFeederMotor.IsOK();
}