// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <subsystems/FeederSubsystem.h>
#include <subsystems/IntakeSubsystem.h>
#include <subsystems/ShooterSubsystem.h>
#include <subsystems/SwerveSubsystem.h>

#include "choreo/auto/AutoFactory.h"
#include "choreo/auto/AutoLoop.h"
#include "choreo/auto/AutoTrajectory.h"

class Autos {
 public:
  frc2::CommandPtr IntakeNote() {
    return frc2::cmd::Sequence(
      frc2::cmd::Parallel(m_intakeSub.IntakeNote(), m_feederSub.Feed()).WithName("IntakeAndFeed").Until([this] { return m_feederSub.HasNote(); }).WithName("IntakeAndFeedUntilNote"),
      frc2::cmd::Print("Got Note!")
    ).WithName("Intake Note Command Auto");
  }

  frc2::CommandPtr CloseFourSafe() {
    mtocenter = factory.Trajectory("CloseFourSafe", 0, safefourloop);
    ctoamp = factory.Trajectory("CloseFourSafe", 1, safefourloop);
    atosource = factory.Trajectory("CloseFourSafe", 2, safefourloop);

    safefourloop.Enabled().OnTrue(
      frc2::cmd::Sequence(
      frc2::cmd::Print("RESET POSE"),
      frc2::cmd::RunOnce([this] { m_swerveSub.ResetPose(mtocenter.GetInitialPose().value()); }).WithName("ResetPoseCmd"),
      frc2::cmd::Print("SPIN UP SUB SHOT"),
      m_shooterSub.RunShooter([] { return consts::shooter::PRESET_SPEEDS::SUBWOOFER; }),
      m_feederSub.Feed().Until([this] { return !m_feederSub.HasNote(); }),
      frc2::cmd::Print("SHOOT"),
      frc2::cmd::Print("CENTER NOTE AND INTAKE"),
      IntakeNote().DeadlineFor(mtocenter.Cmd()),
      m_swerveSub.Stop(),
      frc2::cmd::Print("SPIN UP SHOT"),
      m_shooterSub.RunShooter([] { return consts::shooter::PRESET_SPEEDS::SPEAKER_DIST; }, [this] { return m_swerveSub.GetDistanceToSpeaker(SpeakerSide::CENTER); }),
      m_feederSub.Feed().Until([this] { return !m_feederSub.HasNote(); }),
      frc2::cmd::Print("SHOOT"),
      frc2::cmd::Print("AMP NOTE AND INTAKE"),
      IntakeNote().DeadlineFor(ctoamp.Cmd()),
      m_swerveSub.Stop(),
      frc2::cmd::Print("SPIN UP SHOT"),
      m_shooterSub.RunShooter([] { return consts::shooter::PRESET_SPEEDS::SPEAKER_DIST; }, [this] { return m_swerveSub.GetDistanceToSpeaker(SpeakerSide::AMP_SIDE); }),
      m_feederSub.Feed().Until([this] { return !m_feederSub.HasNote(); }),
      frc2::cmd::Print("SHOOT"),
      frc2::cmd::Print("SOURCE NOTE AND INTAKE"),
      IntakeNote().DeadlineFor(atosource.Cmd()),
      m_swerveSub.Stop(),
      frc2::cmd::Print("SPIN UP SHOT"),
      m_shooterSub.RunShooter([] { return consts::shooter::PRESET_SPEEDS::SPEAKER_DIST; }, [this] { return m_swerveSub.GetDistanceToSpeaker(SpeakerSide::SOURCE); }),
      m_feederSub.Feed().Until([this] { return !m_feederSub.HasNote(); }),
      frc2::cmd::Print("SHOOT")
    ).WithName("WHOLE AUTO COMMAND"));

    return safefourloop.Cmd();
  }

  frc2::CommandPtr TestChoreoAuto() {
    factory.Bind("test", [] { return frc2::cmd::Print("Hello from marker"); });
    straightTraj = factory.Trajectory("ChoreoTest", loop);

    loop.Enabled().OnTrue(frc2::cmd::RunOnce([this] {
                            m_swerveSub.ResetPose(
                                straightTraj.GetInitialPose().value());
                          })
                              .AndThen(straightTraj.Cmd())
                              .WithName("Straight Traj Name"));
    return loop.Cmd().WithName("Test Auto Loop Cmd");
  }

  explicit Autos(SwerveSubsystem& swerveSub, ShooterSubsystem& shooterSub,
                 IntakeSubsystem& intakeSub, FeederSubsystem& feederSub)
      : m_swerveSub{swerveSub},
        m_shooterSub{shooterSub},
        m_intakeSub{intakeSub},
        m_feederSub{feederSub},
        factory{swerveSub.GetFactory()},
        loop{factory.NewLoop("Auto Routine Loops")},
        safefourloop{factory.NewLoop("Safe Four Loop")} {

    BindCommandsAndTriggers();
    
    selectCommand = frc2::cmd::Select<AutoSelector>(
        [this] { return autoChooser.GetSelected(); },
        std::pair{CHOREO_TEST, TestChoreoAuto()},
        std::pair{PP_TEST, pathplanner::PathPlannerAuto("PPTest").ToPtr()},
        std::pair{CLOSE_FOUR_SAFE, CloseFourSafe()});

    autoChooser.AddOption("Choreo Test", AutoSelector::CHOREO_TEST);
    autoChooser.AddOption("Path Planner Test", AutoSelector::PP_TEST);
    autoChooser.AddOption("Close Four Safe", AutoSelector::CLOSE_FOUR_SAFE);

    frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);
  }

  frc2::Command* GetSelectedCommand() { return selectCommand.get(); }

 private:
  void BindCommandsAndTriggers() {
    factory.Bind("SpinUpShooterCenter", [this] { return m_shooterSub.RunShooter(
      [] { 
        return consts::shooter::PRESET_SPEEDS::SPEAKER_DIST; 
      }, 
      [this] {
        return m_swerveSub.GetDistanceToSpeaker(SpeakerSide::CENTER);
      });
      }
    );

    factory.Bind("SpinUpShooterAmp", [this] { return m_shooterSub.RunShooter(
      [] { 
        return consts::shooter::PRESET_SPEEDS::SPEAKER_DIST; 
      }, 
      [this] {
        return m_swerveSub.GetDistanceToSpeaker(SpeakerSide::AMP_SIDE);
      });
      }
    );

    factory.Bind("SpinUpShooterSource", [this] { return m_shooterSub.RunShooter(
      [] { 
        return consts::shooter::PRESET_SPEEDS::SPEAKER_DIST; 
      }, 
      [this] {
        return m_swerveSub.GetDistanceToSpeaker(SpeakerSide::SOURCE);
      });
      }
    );
  }

  enum AutoSelector { CHOREO_TEST, PP_TEST, CLOSE_FOUR_SAFE };

  frc::SendableChooser<AutoSelector> autoChooser;

  SwerveSubsystem& m_swerveSub;
  ShooterSubsystem& m_shooterSub;
  IntakeSubsystem& m_intakeSub;
  FeederSubsystem& m_feederSub;

  choreo::AutoLoop<choreo::SwerveSample> loop;
  choreo::AutoTrajectory<choreo::SwerveSample> straightTraj;

  choreo::AutoLoop<choreo::SwerveSample> safefourloop;
  choreo::AutoTrajectory<choreo::SwerveSample> mtocenter;
  choreo::AutoTrajectory<choreo::SwerveSample> ctoamp;
  choreo::AutoTrajectory<choreo::SwerveSample> atosource;
  choreo::AutoFactory<choreo::SwerveSample>& factory;


  frc2::CommandPtr selectCommand{frc2::cmd::None()};
};
