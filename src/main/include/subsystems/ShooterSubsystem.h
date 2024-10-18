// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/simulation/FlywheelSim.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <networktables/BooleanTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTableInstance.h>

#include <functional>
#include <map>
#include <memory>

#include <ctre/phoenix6/TalonFX.hpp>

#include "constants/ShooterConstants.h"

class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  ShooterSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SimulationPeriodic() override;
  void SetupLUTs(
      const std::map<units::meter_t, consts::shooter::ShooterSpeeds>& speeds);

  frc2::CommandPtr RunShooter(
      std::function<consts::shooter::PRESET_SPEEDS()> preset);
  frc2::CommandPtr RunShooter(
      std::function<consts::shooter::PRESET_SPEEDS()> preset,
      std::function<units::meter_t()> distance);
  frc2::CommandPtr TopWheelSysIdQuasistatic(frc2::sysid::Direction direction);
  frc2::CommandPtr TopWheelSysIdDynamic(frc2::sysid::Direction direction);
  frc2::CommandPtr BottomWheelSysIdQuasistatic(
      frc2::sysid::Direction direction);
  frc2::CommandPtr BottomWheelSysIdDynamic(frc2::sysid::Direction direction);

  frc2::Trigger UpToSpeed() const { return upToSpeedTrigger; }

 private:
  void UpdateNTEntries();

  bool neutralState{true};

  bool IsUpToSpeed() {
    return (units::math::abs(topWheelVelocitySetpoint -
                             currentTopWheelVelocity) <
            consts::shooter::gains::VEL_TOLERANCE) &&
           (units::math::abs(bottomWheelVelocitySetpoint -
                             currentBottomWheelVelocity) <
            consts::shooter::gains::VEL_TOLERANCE) &&
           !neutralState;
  }
  frc2::Trigger upToSpeedTrigger{[this] { return IsUpToSpeed(); }};

  bool ConfigureShooterMotors(bool invertBottom, bool invertTop,
                              units::scalar_t shooterGearing,
                              units::ampere_t supplyCurrentLimit,
                              units::ampere_t statorCurrentLimit);
  bool ConfigureMotorSignals();

  ctre::phoenix6::hardware::TalonFX topWheelMotor{
      consts::shooter::can_ids::TOP_SHOOTER, "rio"};
  ctre::phoenix6::hardware::TalonFX bottomWheelMotor{
      consts::shooter::can_ids::BOTTOM_SHOOTER, "rio"};

  ctre::phoenix6::StatusSignal<units::turn_t> topMotorPosSig =
      topWheelMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> topMotorVelSig =
      topWheelMotor.GetVelocity();
  ctre::phoenix6::StatusSignal<units::volt_t> topMotorVoltageSig =
      topWheelMotor.GetMotorVoltage();
  ctre::phoenix6::controls::VoltageOut topMotorVoltageSetter{0_V};

  ctre::phoenix6::StatusSignal<units::turn_t> bottomMotorPosSig =
      bottomWheelMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> bottomMotorVelSig =
      bottomWheelMotor.GetVelocity();
  ctre::phoenix6::StatusSignal<units::volt_t> bottomMotorVoltageSig =
      bottomWheelMotor.GetMotorVoltage();
  ctre::phoenix6::controls::VoltageOut bottomMotorVoltageSetter{0_V};

  ctre::phoenix6::controls::CoastOut coastSetter{};

  ctre::phoenix6::sim::TalonFXSimState& topMotorSim =
      topWheelMotor.GetSimState();
  ctre::phoenix6::sim::TalonFXSimState& bottomMotorSim =
      bottomWheelMotor.GetSimState();

  frc::LinearSystem<1, 1, 1> shooterPlant{frc::LinearSystemId::FlywheelSystem(
      consts::shooter::physical::SHOOTER_MOTOR,
      consts::shooter::physical::FLYWHEEL_MOI,
      consts::shooter::physical::SHOOTER_RATIO)};

  frc::sim::FlywheelSim topFlywheelSim{
      shooterPlant, consts::shooter::physical::SHOOTER_MOTOR};
  frc::sim::FlywheelSim bottomFlywheelSim{
      shooterPlant, consts::shooter::physical::SHOOTER_MOTOR};

  frc::SimpleMotorFeedforward<units::turns> topWheelFF{
      consts::shooter::gains::SHOOTER_KS, consts::shooter::gains::SHOOTER_KV,
      consts::shooter::gains::SHOOTER_KA};

  frc::SimpleMotorFeedforward<units::turns> bottomWheelFF{
      consts::shooter::gains::SHOOTER_KS, consts::shooter::gains::SHOOTER_KV,
      consts::shooter::gains::SHOOTER_KA};

  frc::PIDController topWheelPID{consts::shooter::gains::SHOOTER_KP.value(),
                                 consts::shooter::gains::SHOOTER_KI.value(),
                                 consts::shooter::gains::SHOOTER_KD.value()};

  frc::PIDController bottomWheelPID{consts::shooter::gains::SHOOTER_KP.value(),
                                    consts::shooter::gains::SHOOTER_KI.value(),
                                    consts::shooter::gains::SHOOTER_KD.value()};

  frc2::sysid::SysIdRoutine topWheelSysIdRoutine{
      frc2::sysid::Config{std::nullopt, std::nullopt, std::nullopt, nullptr},
      frc2::sysid::Mechanism{
          [this](units::volt_t driveVoltage) {
            topWheelMotor.SetControl(
                topMotorVoltageSetter.WithOutput(driveVoltage));
          },
          [this](frc::sysid::SysIdRoutineLog* log) {
            log->Motor("top-shooter-wheel")
                .voltage(topMotorVoltageSig.GetValue())
                .position(currentTopWheelPosition)
                .velocity(currentTopWheelVelocity);
          },
          this}};

  frc2::sysid::SysIdRoutine bottomWheelSysIdRoutine{
      frc2::sysid::Config{std::nullopt, std::nullopt, std::nullopt, nullptr},
      frc2::sysid::Mechanism{
          [this](units::volt_t driveVoltage) {
            bottomWheelMotor.SetControl(
                bottomMotorVoltageSetter.WithOutput(driveVoltage));
          },
          [this](frc::sysid::SysIdRoutineLog* log) {
            log->Motor("bottom-shooter-wheel")
                .voltage(bottomMotorVoltageSig.GetValue())
                .position(currentBottomWheelPosition)
                .velocity(currentBottomWheelVelocity);
          },
          this}};

  std::shared_ptr<nt::NetworkTable> nt{
      nt::NetworkTableInstance::GetDefault().GetTable("Shooter")};
  nt::DoublePublisher topWheelVelocityPub{
      nt->GetDoubleTopic("TopWheelVelocityRPM").Publish()};
  nt::DoublePublisher topWheelSetpointPub{
      nt->GetDoubleTopic("TopWheelSetpointRPM").Publish()};
  nt::DoublePublisher bottomWheelVelocityPub{
      nt->GetDoubleTopic("BottomWheelVelocityRPM").Publish()};
  nt::DoublePublisher bottomWheelSetpointPub{
      nt->GetDoubleTopic("BottomWheelSetpointRPM").Publish()};
  nt::BooleanPublisher isUpToSpeedPub{
      nt->GetBooleanTopic("IsUpToSpeed").Publish()};
  nt::DoubleEntry topWheelTuningSetpoint{nt->GetDoubleTopic("TopWheelTuningSetpoint").GetEntry(0)};
  nt::DoubleEntry bottomWheelTuningSetpoint{nt->GetDoubleTopic("BottomWheelTuningSetpoint").GetEntry(0)};

  units::turns_per_second_t topWheelVelocitySetpoint{0_rpm};
  units::turns_per_second_t bottomWheelVelocitySetpoint{0_rpm};
  units::turn_t currentTopWheelPosition{0_rad};
  units::turn_t currentBottomWheelPosition{0_rad};
  units::turns_per_second_t currentTopWheelVelocity{0_rpm};
  units::turns_per_second_t currentBottomWheelVelocity{0_rpm};
  bool runningSysid{false};
};
