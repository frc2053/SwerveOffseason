// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <constants/FeederConstants.h>
#include <frc/system/plant/LinearSystemId.h>
#include <networktables/DoubleTopic.h>
#include <frc/simulation/FlywheelSim.h>
#include <networktables/NetworkTableInstance.h>

class FeederSubsystem : public frc2::SubsystemBase {
 public:
  FeederSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SimulationPeriodic() override;

 private:
  bool ConfigureFeederMotor(bool invert,
                            units::scalar_t intakeGearing,
                            units::ampere_t supplyCurrentLimit,
                            units::ampere_t statorCurrentLimit);
  bool ConfigureMotorSignals();
  void UpdateNTEntries();

  ctre::phoenix6::hardware::TalonFX feederMotor{consts::feeder::can_ids::FEEDER, "*"};

  ctre::phoenix6::StatusSignal<units::volt_t> feederMotorVoltageSig = feederMotor.GetMotorVoltage();
  ctre::phoenix6::StatusSignal<units::ampere_t> feederMotorTorqueCurrentSig = feederMotor.GetTorqueCurrent();
  ctre::phoenix6::controls::VoltageOut feederMotorVoltageSetter{0_V};
  
    ctre::phoenix6::sim::TalonFXSimState &feederMotorSim = feederMotor.GetSimState();

  frc::LinearSystem<1, 1, 1>  feederPlant{frc::LinearSystemId::FlywheelSystem(
      consts::feeder::physical::FEEDER_MOTOR, consts::feeder::physical::FEEDER_MOI, consts::feeder::physical::FEEDER_RATIO)};

  frc::sim::FlywheelSim feederSim{feederPlant, consts::feeder::physical::FEEDER_MOTOR};

  std::shared_ptr<nt::NetworkTable> nt{
      nt::NetworkTableInstance::GetDefault().GetTable("Feeder")};
  nt::DoublePublisher feederWheelMotorVoltagePub{
    nt->GetDoubleTopic("FeederWheelMotorVoltage").Publish()
  };
  nt::DoublePublisher feederWheelMotorVoltageSetpointPub{
    nt->GetDoubleTopic("FeederWheelMotorVoltageSetpoint").Publish()
  };
  nt::DoublePublisher hasNotePub{
    nt->GetDoubleTopic("HasNote").Publish()
  };

  bool hasNote{false};

  units::volt_t feederWheelVoltageSetpoint{0_V};
  units::volt_t currentFeederWheelVoltage{0_V};

};
