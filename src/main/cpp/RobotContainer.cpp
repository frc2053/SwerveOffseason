// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "RobotContainer.h"

#include <frc/MathUtil.h>
#include <frc/RobotBase.h>
#include <frc2/command/Commands.h>
#include <str/DriverstationUtils.h>
#include <frc2/command/button/RobotModeTriggers.h>

RobotContainer::RobotContainer() {
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // DEFAULT DRIVE COMMAND
  swerveSubsystem.SetDefaultCommand(swerveSubsystem.Drive(
      [this] {
        return str::NegateIfRed(
            frc::ApplyDeadband<double>(-driverController.GetLeftY(), .1) *
            consts::swerve::physical::DRIVE_MAX_SPEED);
      },
      [this] {
        return str::NegateIfRed(
            frc::ApplyDeadband<double>(-driverController.GetLeftX(), .1) *
            consts::swerve::physical::DRIVE_MAX_SPEED);
      },
      [this] {
        return frc::ApplyDeadband<double>(-driverController.GetRightX(), .1) *
               consts::swerve::physical::DRIVE_MAX_ROT_SPEED;
      },
      true, true));

  // FAKE NOTE INTAKE
  if (frc::RobotBase::IsSimulation()) {
    driverController.Start().WhileTrue(intakeSubsystem.FakeNote());
  }

  driverController.LeftBumper().WhileTrue(swerveSubsystem.FaceSpeaker(
    [this] {
      return str::NegateIfRed(
          frc::ApplyDeadband<double>(-driverController.GetLeftY(), .1) *
          consts::swerve::physical::DRIVE_MAX_SPEED);
    },
    [this] {
      return str::NegateIfRed(
          frc::ApplyDeadband<double>(-driverController.GetLeftX(), .1) *
          consts::swerve::physical::DRIVE_MAX_SPEED);
    }
  ));

  driverController.POVDown().WhileTrue(climberSubsystem.ManualControl([] { return .6; }, [] { return -.6; }));

  (driverController.LeftTrigger() && frc2::RobotModeTriggers::Teleop()).OnTrue(IntakeNote());
  (!driverController.LeftTrigger() && !intakeSubsystem.TouchedNote() && frc2::RobotModeTriggers::Teleop())
      .OnTrue(frc2::cmd::Sequence(intakeSubsystem.Stop(),
                                  frc2::cmd::Print("Cancelled")));

  (shooterSubsystem.UpToSpeed() && driverController.RightTrigger() && frc2::RobotModeTriggers::Teleop())
      .OnTrue(feederSubsystem.Feed())
      .OnFalse(feederSubsystem.Stop());

  (intakeSubsystem.TouchedNote() && frc2::RobotModeTriggers::Teleop()).OnTrue(RumbleDriver([] { return .1_s; }));
  (feederSubsystem.GotNote() && frc2::RobotModeTriggers::Teleop()).OnTrue(frc2::cmd::Parallel(
      intakeSubsystem.Stop(), feederSubsystem.Stop(),
      RumbleDriver([] { return .5_s; }), RumbleOperator([] { return .5_s; })));

  driverController.A().WhileTrue(swerveSubsystem.AlignToAmp());

  driverController.Start().OnTrue(swerveSubsystem.ZeroYaw());


  // controller.A().WhileTrue(swerveSubsystem.SysIdDriveQuasistaticTorque(frc2::sysid::Direction::kForward));
  // controller.B().WhileTrue(swerveSubsystem.SysIdDriveQuasistaticTorque(frc2::sysid::Direction::kReverse));
  // controller.X().WhileTrue(swerveSubsystem.SysIdDriveDynamicTorque(frc2::sysid::Direction::kForward));
  // controller.Y().WhileTrue(swerveSubsystem.SysIdDriveDynamicTorque(frc2::sysid::Direction::kReverse));

  // driverController.POVDown().OnTrue(swerveSubsystem.TuneSteerPID([this] { return
  // driverController.Start().Get(); }));
  // driverController.POVUp().OnTrue(swerveSubsystem.TuneDrivePID([this] { return
  // driverController.Start().Get(); }));
  // driverController.POVLeft().OnTrue(swerveSubsystem.TuneAnglePID([this] { return
  // driverController.Start().Get(); }));
  // driverController.POVRight().OnTrue(swerveSubsystem.TunePosePID([this] { return
  // driverController.Start().Get(); }));

  tuningTable->PutBoolean("WheelRadiusFwd", false);
  tuningTable->PutBoolean("WheelRadiusRev", false);


  wheelRadFwdBtn.WhileTrue(swerveSubsystem.WheelRadius(frc2::sysid::Direction::kForward));
  wheelRadRevBtn.WhileTrue(swerveSubsystem.WheelRadius(frc2::sysid::Direction::kReverse));

  operatorController.A().WhileTrue(shooterSubsystem.RunShooter(
      [] { return consts::shooter::PRESET_SPEEDS::AMP; }));
  operatorController.A().OnFalse(shooterSubsystem.RunShooter(
      [] { return consts::shooter::PRESET_SPEEDS::OFF; }));

  operatorController.Y().WhileTrue(shooterSubsystem.RunShooter(
      [] { return consts::shooter::PRESET_SPEEDS::PASS; }));
  operatorController.Y().OnFalse(shooterSubsystem.RunShooter(
      [] { return consts::shooter::PRESET_SPEEDS::OFF; }));

  operatorController.B().WhileTrue(shooterSubsystem.RunShooter(
      [] { return consts::shooter::PRESET_SPEEDS::SUBWOOFER; }));
  operatorController.B().OnFalse(shooterSubsystem.RunShooter(
      [] { return consts::shooter::PRESET_SPEEDS::OFF; }));

  operatorController.X().OnTrue(shooterSubsystem.RunShooter(
      [] { return consts::shooter::PRESET_SPEEDS::SPEAKER_DIST; }, [this] { return swerveSubsystem.GetDistanceToSpeaker(swerveSubsystem.GetSpeakerSideFromPosition()); }).Repeatedly());
  operatorController.X().OnFalse(shooterSubsystem.RunShooter(
      [] { return consts::shooter::PRESET_SPEEDS::OFF; }));

  operatorController.Start().WhileTrue(shooterSubsystem.RunShooter([] { return consts::shooter::PRESET_SPEEDS::TUNING; }));
  operatorController.Start().OnFalse(shooterSubsystem.RunShooter([] { return consts::shooter::PRESET_SPEEDS::OFF; }));

  operatorController.Back().WhileTrue(
      frc2::cmd::Parallel(intakeSubsystem.PoopNote(), feederSubsystem.Eject()));

  // operatorController.A().WhileTrue(shooterSubsystem.TopWheelSysIdQuasistatic(frc2::sysid::Direction::kForward));
  // operatorController.B().WhileTrue(shooterSubsystem.TopWheelSysIdQuasistatic(frc2::sysid::Direction::kReverse));
  // operatorController.X().WhileTrue(shooterSubsystem.TopWheelSysIdDynamic(frc2::sysid::Direction::kForward));
  // operatorController.Y().WhileTrue(shooterSubsystem.TopWheelSysIdDynamic(frc2::sysid::Direction::kReverse));

  // (operatorController.A() &&
  // operatorController.Back()).WhileTrue(shooterSubsystem.BottomWheelSysIdQuasistatic(frc2::sysid::Direction::kForward));
  // (operatorController.B() &&
  // operatorController.Back()).WhileTrue(shooterSubsystem.BottomWheelSysIdQuasistatic(frc2::sysid::Direction::kReverse));
  // (operatorController.X() &&
  // operatorController.Back()).WhileTrue(shooterSubsystem.BottomWheelSysIdDynamic(frc2::sysid::Direction::kForward));
  // (operatorController.Y() &&
  // operatorController.Back()).WhileTrue(shooterSubsystem.BottomWheelSysIdDynamic(frc2::sysid::Direction::kReverse));

  // operatorController.A().WhileTrue(swerveSubsystem.SysIdSteerMk4iQuasistaticTorque(frc2::sysid::Direction::kForward));
  // operatorController.B().WhileTrue(swerveSubsystem.SysIdSteerMk4iQuasistaticTorque(frc2::sysid::Direction::kReverse));
  // operatorController.X().WhileTrue(swerveSubsystem.SysIdSteerMk4iDynamicTorque(frc2::sysid::Direction::kForward));
  // operatorController.Y().WhileTrue(swerveSubsystem.SysIdSteerMk4iDynamicTorque(frc2::sysid::Direction::kReverse));

  // driverController.A().WhileTrue(swerveSubsystem.SysIdSteerMk4iQuasistaticVoltage(frc2::sysid::Direction::kForward));
  // driverController.B().WhileTrue(swerveSubsystem.SysIdSteerMk4iQuasistaticVoltage(frc2::sysid::Direction::kReverse));
  // driverController.X().WhileTrue(swerveSubsystem.SysIdSteerMk4iDynamicVoltage(frc2::sysid::Direction::kForward));
  // driverController.Y().WhileTrue(swerveSubsystem.SysIdSteerMk4iDynamicVoltage(frc2::sysid::Direction::kReverse));

  // controller.A().WhileTrue(swerveSubsystem.SysIdDriveQuasistaticVoltage(frc2::sysid::Direction::kForward));
  // controller.B().WhileTrue(swerveSubsMk4iystem.SysIdDriveQuasistaticVoltage(frc2::sysid::Direction::kReverse));
  // controller.X().WhileTrue(swerveSubsystem.SysIdDriveDynamicVoltage(frc2::sysid::Direction::kForward));
  // controller.Y().WhileTrue(swerveSubsystem.SysIdDriveDynamicVoltage(frc2::sysid::Direction::kReverse));

  // controller.A().WhileTrue(swerveSubsystem.PointWheelsToAngle([] { return
  // 0_deg; })); controller.B().WhileTrue(swerveSubsystem.PointWheelsToAngle([]
  // { return 45_deg; }));
  // controller.X().WhileTrue(swerveSubsystem.PointWheelsToAngle([] { return
  // 90_deg; })); controller.Y().WhileTrue(swerveSubsystem.PointWheelsToAngle([]
  // { return 180_deg; }));
}

frc2::CommandPtr RobotContainer::IntakeNote() {
  return frc2::cmd::Parallel(frc2::cmd::Print("IntakeNote"),
                             intakeSubsystem.IntakeNote(),
                             feederSubsystem.Feed())
      .Until([this] { return feederSubsystem.HasNote(); });
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return autos.GetSelectedCommand();
}

SwerveSubsystem& RobotContainer::GetSwerveSubsystem() {
  return swerveSubsystem;
}

ShooterSubsystem& RobotContainer::GetShooterSubsystem() {
  return shooterSubsystem;
}

IntakeSubsystem& RobotContainer::GetIntakeSubsystem() {
  return intakeSubsystem;
}

FeederSubsystem& RobotContainer::GetFeederSubsystem() {
  return feederSubsystem;
}

str::Vision &RobotContainer::GetVision() { return vision; }

str::NoteVisualizer& RobotContainer::GetNoteVisualizer() {
  return noteVisualizer;
}

frc2::CommandPtr RobotContainer::RumbleDriver(
    std::function<units::second_t()> timeToRumble) {
  return frc2::cmd::Sequence(
             frc2::cmd::RunOnce([this] {
               driverController.SetRumble(
                   frc::GenericHID::RumbleType::kBothRumble, 1.0);
             }),
             frc2::cmd::Wait(timeToRumble()), frc2::cmd::RunOnce([this] {
               driverController.SetRumble(
                   frc::GenericHID::RumbleType::kBothRumble, 0.0);
             }))
      .FinallyDo([this] {
        driverController.SetRumble(frc::GenericHID::RumbleType::kBothRumble,
                                   0.0);
      });
}

frc2::CommandPtr RobotContainer::RumbleOperator(
    std::function<units::second_t()> timeToRumble) {
  return frc2::cmd::Sequence(
             frc2::cmd::RunOnce([this] {
               operatorController.SetRumble(
                   frc::GenericHID::RumbleType::kBothRumble, 1.0);
             }),
             frc2::cmd::Wait(timeToRumble()), frc2::cmd::RunOnce([this] {
               operatorController.SetRumble(
                   frc::GenericHID::RumbleType::kBothRumble, 0.0);
             }))
      .FinallyDo([this] {
        operatorController.SetRumble(frc::GenericHID::RumbleType::kBothRumble,
                                     0.0);
      });
}
