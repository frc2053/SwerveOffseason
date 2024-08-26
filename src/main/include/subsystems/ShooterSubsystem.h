// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/simulation/FlywheelSim.h>
#include <frc/system/plant/LinearSystemId.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTableInstance.h>

#include "constants/ShooterConstants.h"

class ShooterSubsystem : public frc2::SubsystemBase {
public:
  ShooterSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SimulationPeriodic() override;
  bool IsUpToSpeed();

private:
  bool ConfigureShooterMotors(bool invertBottom, bool invertTop,
                              units::scalar_t shooterGearing,
                              units::ampere_t supplyCurrentLimit,
                              units::ampere_t statorCurrentLimit);
  bool ConfigureMotorSignals();

  ctre::phoenix6::hardware::TalonFX topWheelMotor{consts::shooter::can_ids::TOP_SHOOTER, "*"};
  ctre::phoenix6::hardware::TalonFX bottomWheelMotor{consts::shooter::can_ids::BOTTOM_SHOOTER, "*"};

  ctre::phoenix6::StatusSignal<units::turns_per_second_t> topMotorVelSig = topWheelMotor.GetVelocity();
  ctre::phoenix6::StatusSignal<units::volt_t> topMotorVoltageSig = topWheelMotor.GetMotorVoltage();
  ctre::phoenix6::controls::VoltageOut topMotorSetter{0_V};

  ctre::phoenix6::StatusSignal<units::turns_per_second_t> bottomMotorVelSig = bottomWheelMotor.GetVelocity();
  ctre::phoenix6::StatusSignal<units::volt_t> bottomMotorVoltageSig = bottomWheelMotor.GetMotorVoltage();
  ctre::phoenix6::controls::VoltageOut bottomMotorSetter{0_V};

  ctre::phoenix6::sim::TalonFXSimState &topMotorSim = topWheelMotor.GetSimState();
  ctre::phoenix6::sim::TalonFXSimState &bottomMotorSim = bottomWheelMotor.GetSimState();

  frc::LinearSystem<1, 1, 1> shooterPlant{frc::LinearSystemId::FlywheelSystem(
      consts::shooter::physical::SHOOTER_MOTOR, consts::shooter::physical::FLYWHEEL_MOI, consts::shooter::physical::SHOOTER_RATIO)};

  frc::sim::FlywheelSim topFlywheelSim{shooterPlant, consts::shooter::physical::SHOOTER_MOTOR};
  frc::sim::FlywheelSim bottomFlywheelSim{shooterPlant, consts::shooter::physical::SHOOTER_MOTOR};

  std::shared_ptr<nt::NetworkTable> nt{
      nt::NetworkTableInstance::GetDefault().GetTable("Shooter")};
  nt::DoublePublisher loopTimePub{
      nt::NetworkTableInstance::GetDefault()
          .GetTable("Metadata")
          ->GetDoubleTopic("ShooterOdomLoopRate")
          .Publish()};
  nt::DoublePublisher topWheelVelocityPub{
    nt->GetDoubleTopic("TopWheelVelocityRPM").Publish()
  };
  nt::DoublePublisher topWheelSetpointPub{
    nt->GetDoubleTopic("TopWheelSetpointRPM").Publish()
  };
  nt::DoublePublisher bottomWheelVelocityPub{
    nt->GetDoubleTopic("BottomWheelVelocityRPM").Publish()
  };
  nt::DoublePublisher bottomWheelSetpointPub{
    nt->GetDoubleTopic("BottomWheelSetpointRPM").Publish()
  };

  units::radians_per_second_t topWheelVelocitySetpoint{0_rpm};
  units::radians_per_second_t bottomWheelVelocitySetpoint{0_rpm};
  units::radians_per_second_t currentTopWheelVelocity{0_rpm};
  units::radians_per_second_t currentBottomWheelVelocity{0_rpm};
};
