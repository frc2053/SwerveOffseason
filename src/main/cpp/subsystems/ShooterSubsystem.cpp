// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "subsystems/ShooterSubsystem.h"
#include "constants/Constants.h"
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>


ShooterSubsystem::ShooterSubsystem() {
  ConfigureShooterMotors(consts::shooter::physical::BOTTOM_INVERT,
                         consts::shooter::physical::TOP_INVERT,
                         consts::shooter::physical::SHOOTER_RATIO,
                         consts::shooter::current_limits::SUPPLY_CURRENT_LIMIT,
                         consts::shooter::current_limits::STATOR_CURRENT_LIMIT);
  ConfigureMotorSignals();
  SetName("ShooterSubsystem");
  frc::SmartDashboard::PutData(this);
}

frc2::CommandPtr ShooterSubsystem::RunShooter(std::function<consts::shooter::PRESET_SPEEDS()> preset) {
    return frc2::cmd::Run([this, preset] {
        switch(preset()) {
            case consts::shooter::PRESET_SPEEDS::AMP:
                topWheelVelocitySetpoint = consts::shooter::AMP_SPEEDS.topSpeed;
                bottomWheelVelocitySetpoint = consts::shooter::AMP_SPEEDS.bottomSpeed;
                break;
            case consts::shooter::PRESET_SPEEDS::OFF:
                topWheelVelocitySetpoint = 0_rpm;
                bottomWheelVelocitySetpoint = 0_rpm;
                break;
            default:
                topWheelVelocitySetpoint = 0_rpm;
                bottomWheelVelocitySetpoint = 0_rpm;
                break;
        }
    }, {this}).Until([this] { return IsUpToSpeed(); });
}

// This method will be called once per scheduler run
void ShooterSubsystem::Periodic() {
    ctre::phoenix::StatusCode shooterWaitResult = ctre::phoenix6::BaseStatusSignal::RefreshAll({
        &topMotorPosSig, 
        &topMotorVelSig, 
        &topMotorVoltageSig,
        &bottomMotorPosSig,
        &bottomMotorVelSig, 
        &bottomMotorVoltageSig
    });

    if(!shooterWaitResult.IsOK()) {
        fmt::print("Error grabbing shooter signals! Details: {}\n", shooterWaitResult.GetName());
    }

    currentTopWheelPosition = ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(topMotorPosSig, topMotorVelSig);
    currentBottomWheelPosition = ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(bottomMotorPosSig, bottomMotorVelSig);

    currentBottomWheelVelocity = bottomMotorVelSig.GetValue();
    currentTopWheelVelocity = topMotorVelSig.GetValue();

    units::volt_t topFFVolts = topWheelFF.Calculate(topWheelVelocitySetpoint);
    units::volt_t bottomFFVolts = bottomWheelFF.Calculate(bottomWheelVelocitySetpoint);

    units::volt_t topPIDOutput = units::volt_t{topWheelPID.Calculate(currentTopWheelVelocity.value(), topWheelVelocitySetpoint.value())};
    units::volt_t bottomPIDOutput = units::volt_t{bottomWheelPID.Calculate(currentTopWheelVelocity.value(), topWheelVelocitySetpoint.value())};

    if(!runningSysid) {
        if(topWheelVelocitySetpoint == 0_rpm) {
            topWheelMotor.SetControl(coastSetter);
        }
        else {
            topWheelMotor.SetControl(topMotorVoltageSetter.WithOutput(topFFVolts + topPIDOutput));
        }
        if(bottomWheelVelocitySetpoint == 0_rpm) {
            bottomWheelMotor.SetControl(coastSetter);
        }
        else {
            bottomWheelMotor.SetControl(bottomMotorVoltageSetter.WithOutput(bottomFFVolts + bottomPIDOutput));
        }
    }

    UpdateNTEntries();
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

void ShooterSubsystem::UpdateNTEntries() {
    topWheelSetpointPub.Set(topWheelVelocitySetpoint.convert<units::revolutions_per_minute>().value());
    bottomWheelSetpointPub.Set(bottomWheelVelocitySetpoint.convert<units::revolutions_per_minute>().value());

    bottomWheelVelocityPub.Set(currentBottomWheelVelocity.convert<units::revolutions_per_minute>().value());
    topWheelVelocityPub.Set(currentTopWheelVelocity.convert<units::revolutions_per_minute>().value());

    isUpToSpeedPub.Set(UpToSpeed().Get());
}

bool ShooterSubsystem::ConfigureShooterMotors(
    bool invertBottom, bool invertTop, units::scalar_t shooterGearing,
    units::ampere_t supplyCurrentLimit, units::ampere_t statorCurrentLimit) {

  ctre::phoenix6::configs::TalonFXConfiguration shooterConfig{};

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
  topMotorVoltageSetter.UpdateFreqHz = 0_Hz;
  bottomMotorVoltageSetter.UpdateFreqHz = 0_Hz;
  coastSetter.UpdateFreqHz = 0_Hz;

  //Double the rio update rate? Not sure what is optimal here
  units::hertz_t updateRate = 1.0 / (consts::LOOP_PERIOD * 2.0);

  topMotorPosSig.SetUpdateFrequency(updateRate);
  topMotorVelSig.SetUpdateFrequency(updateRate);
  topMotorVoltageSig.SetUpdateFrequency(updateRate);
  bottomMotorPosSig.SetUpdateFrequency(updateRate);
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

frc2::CommandPtr ShooterSubsystem::TopWheelSysIdQuasistatic(frc2::sysid::Direction direction) {
  return topWheelSysIdRoutine.Quasistatic(direction).BeforeStarting([this] { runningSysid = true; }).AndThen([this] { runningSysid = false; });
}

frc2::CommandPtr ShooterSubsystem::TopWheelSysIdDynamic(frc2::sysid::Direction direction) {
  return topWheelSysIdRoutine.Dynamic(direction).BeforeStarting([this] { runningSysid = true; }).AndThen([this] { runningSysid = false; });;
}

frc2::CommandPtr ShooterSubsystem::BottomWheelSysIdQuasistatic(frc2::sysid::Direction direction) {
  return bottomWheelSysIdRoutine.Quasistatic(direction).BeforeStarting([this] { runningSysid = true; }).AndThen([this] { runningSysid = false; });
}

frc2::CommandPtr ShooterSubsystem::BottomWheelSysIdDynamic(frc2::sysid::Direction direction) {
  return bottomWheelSysIdRoutine.Dynamic(direction).BeforeStarting([this] { runningSysid = true; }).AndThen([this] { runningSysid = false; });;
}