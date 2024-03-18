// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <str/SwerveDrive.h>

class SwerveSubsystem : public frc2::SubsystemBase {
 public:
  SwerveSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SimulationPeriodic() override;
  units::ampere_t GetSimulatedCurrentDraw() const;
  void UpdateSwerveOdom();
  frc2::CommandPtr SysIdSteerQuasistatic(frc2::sysid::Direction dir);
  frc2::CommandPtr SysIdSteerDynamic(frc2::sysid::Direction dir);

 private:

  str::SwerveDrive swerveDrive;

  //It says volts, because sysid only supports volts for now. But we are using current anyway
  frc2::sysid::SysIdRoutine steerTorqueSysid{
    frc2::sysid::Config{
      10_V / 1_s,
      30_V,
      10_s,
      [this](frc::sysid::State state) {
        ctre::phoenix6::SignalLogger().WriteString(
            "state", frc::sysid::SysIdRoutineLog::StateEnumToString(state));
      }
    },
    frc2::sysid::Mechanism{
      [this](units::volt_t voltsToSend) {
        modules[0].SetSteerToTorque(voltsToSend);
      },
      [this](frc::sysid::SysIdRoutineLog* log) {
        modules[0].LogSteerTorqueSysId(log);
      },
      this,
      "swerve-steer"
    }
  };
};
