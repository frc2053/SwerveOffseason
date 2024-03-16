// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "str/SwerveModule.h"
#include "constants/SwerveConstants.h"

class SwerveSubsystem : public frc2::SubsystemBase {
 public:
  SwerveSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SimulationPeriodic() override;
  units::ampere_t GetCurrentDraw() const;
  void UpdateSwerveOdom();
  frc2::CommandPtr SysIdSteerQuasistatic(frc2::sysid::Direction dir);
  frc2::CommandPtr SysIdSteerDynamic(frc2::sysid::Direction dir);

 private:
  str::SwerveModule testModule{
    str::SwerveModuleConstants{
      "test",
      consts::swerve::can_ids::FL_STEER,
      consts::swerve::can_ids::FL_DRIVE,
      consts::swerve::can_ids::FL_ENC,
      consts::swerve::physical::FL_ENC_OFFSET,
      consts::swerve::physical::FL_DRIVE_INVERT,
      consts::swerve::physical::FL_STEER_INVERT
    },
    str::SwerveModulePhysical{
      consts::swerve::physical::STEER_GEARING,
      consts::swerve::physical::DRIVE_GEARING,
      consts::swerve::current_limits::SUPPLY_CURRENT_LIMIT,
      consts::swerve::current_limits::SLIP_CURRENT_LIMIT,
      consts::swerve::physical::DRIVE_MOTOR,
      consts::swerve::physical::STEER_MOTOR,
      consts::swerve::physical::COUPLING_RATIO,
      consts::swerve::physical::WHEEL_RADIUS,
      consts::swerve::gains::DRIVE_KS_V,
      consts::swerve::gains::STEER_KS_V,
    },
    str::SwerveModuleSteerGains{
      consts::swerve::gains::STEER_CRUISE_VEL,
      consts::swerve::gains::STEER_MOTION_MAGIC_KA,
      consts::swerve::gains::STEER_MOTION_MAGIC_KV,
      consts::swerve::gains::STEER_KA,
      consts::swerve::gains::STEER_KV,
      consts::swerve::gains::STEER_KS,
      consts::swerve::gains::STEER_KP,
      consts::swerve::gains::STEER_KI,
      consts::swerve::gains::STEER_KD,
    },
    str::SwerveModuleDriveGains{
      consts::swerve::gains::DRIVE_KA,
      consts::swerve::gains::DRIVE_KV,
      consts::swerve::gains::DRIVE_KS,
      consts::swerve::gains::DRIVE_KP,
      consts::swerve::gains::DRIVE_KI,
      consts::swerve::gains::DRIVE_KD,
    }
  };
  
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
        testModule.SetSteerToTorque(voltsToSend);
      },
      [this](frc::sysid::SysIdRoutineLog* log) {
        testModule.LogSteerTorqueSysId(log);
      },
      this,
      "swerve-steer"
    }
  };

  units::second_t lastLoopTime;

  std::array<ctre::phoenix6::BaseStatusSignal*, 6> allSignals;
};
