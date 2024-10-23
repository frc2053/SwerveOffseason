// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "subsystems/SwerveSubsystem.h"

#include <frc/DataLogManager.h>
#include <frc/Filesystem.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/FlippingUtil.h>
#include <str/Math.h>

#include <memory>
#include <string>
#include <vector>

#include "constants/Constants.h"
#include "constants/VisionConstants.h"

SwerveSubsystem::SwerveSubsystem() {
  SetName("SwerveSubsystem");
  frc::SmartDashboard::PutData(this);
  SetupPathplanner();

  rotationController.EnableContinuousInput(-std::numbers::pi, std::numbers::pi);
}

void SwerveSubsystem::UpdateSwerveOdom() {
  swerveDrive.UpdateSwerveOdom();
}

void SwerveSubsystem::AddVisionMeasurement(const frc::Pose2d& visionMeasurement,
                                           units::second_t timestamp,
                                           const Eigen::Vector3d& stdDevs) {
  swerveDrive.AddVisionMeasurement(visionMeasurement, timestamp, stdDevs);
}

// This method will be called once per scheduler run
void SwerveSubsystem::Periodic() {
  swerveDrive.UpdateNTEntries();
  frc::SmartDashboard::PutNumber("Distance to Speaker Center", GetDistanceToSpeaker(SpeakerSide::CENTER).convert<units::inches>().value());
  frc::SmartDashboard::PutNumber("Distance to Speaker Amp", GetDistanceToSpeaker(SpeakerSide::AMP_SIDE).convert<units::inches>().value());
  frc::SmartDashboard::PutNumber("Distance to Speaker Source", GetDistanceToSpeaker(SpeakerSide::SOURCE).convert<units::inches>().value());
}

void SwerveSubsystem::SimulationPeriodic() {
  swerveDrive.SimulationPeriodic();
}

frc::Pose2d SwerveSubsystem::GetOdomPose() {
  return swerveDrive.GetOdomPose();
}

frc::Pose2d SwerveSubsystem::GetRobotPose() {
  return swerveDrive.GetPose();
}

frc::ChassisSpeeds SwerveSubsystem::GetFieldRelativeSpeed() {
  return frc::ChassisSpeeds::FromRobotRelativeSpeeds(
      swerveDrive.GetRobotRelativeSpeeds(), GetRobotPose().Rotation());
}

frc::ChassisSpeeds SwerveSubsystem::GetRobotRelativeSpeed() const {
  return swerveDrive.GetRobotRelativeSpeeds();
}

units::ampere_t SwerveSubsystem::GetSimulatedCurrentDraw() const {
  return swerveDrive.GetSimulatedCurrentDraw();
}

SpeakerSide SwerveSubsystem::GetSpeakerSideFromPosition() {
    frc::Translation2d speakerLocation = consts::yearSpecific::speakerLocationCenter;

    frc::Pose2d robotPose = GetRobotPose();
    frc::Translation2d robotPosition = robotPose.Translation();
    frc::Translation2d line1Point{0_m, 5.38_m};
    frc::Rotation2d line1Dir{30_deg};
    frc::Translation2d line2Point{0_m, 5.70_m};
    frc::Rotation2d line2Dir{-30_deg};

    bool isInCenter = false;
    bool isAmpSide = false;
    bool isSourceSide = false;
    if (str::IsOnRed()) {
      speakerLocation = pathplanner::FlippingUtil::flipFieldPosition(speakerLocation);
      line1Point = pathplanner::FlippingUtil::flipFieldPosition(line1Point);
      line2Point = pathplanner::FlippingUtil::flipFieldPosition(line2Point);
      isInCenter = str::math::LineBetweenChecker::IsBetweenLines(robotPosition, line1Point, -line1Dir, line2Point, -line2Dir);
      isAmpSide = str::math::LineBetweenChecker::IsPastLine(robotPosition, line2Point, -line2Dir, true);
      isSourceSide = str::math::LineBetweenChecker::IsPastLine(robotPosition, line1Point, -line1Dir, false);
    }
    else {
      isInCenter = str::math::LineBetweenChecker::IsBetweenLines(robotPosition, line1Point, line1Dir, line2Point, line2Dir);
      isAmpSide = str::math::LineBetweenChecker::IsPastLine(robotPosition, line2Point, line2Dir, true);
      isSourceSide = str::math::LineBetweenChecker::IsPastLine(robotPosition, line1Point, line1Dir, false);
    }

    if(!isInCenter) {
      if(isAmpSide) {
        return SpeakerSide::SOURCE;
      } else if (isSourceSide) {
        return SpeakerSide::AMP_SIDE;
      }
    }
    else {
      return SpeakerSide::CENTER;
    }
}

frc2::CommandPtr SwerveSubsystem::RotateToAngle(
  std::function<units::meters_per_second_t()> xVel, 
  std::function<units::meters_per_second_t()> yVel, 
  std::function<units::radian_t()> goalAngle, 
  std::function<bool()> override) {
  return frc2::cmd::Sequence(
  frc2::cmd::RunOnce([this] {
    thetaController.EnableContinuousInput(
        units::radian_t{-std::numbers::pi},
        units::radian_t{std::numbers::pi});
    thetaController.SetTolerance(
        consts::swerve::pathplanning::rotationalPIDTolerance,
        consts::swerve::pathplanning::rotationalVelPIDTolerance);
  }, {this}),
  frc2::cmd::Run([this, xVel, yVel, goalAngle] {

    units::radian_t goal = goalAngle();
    frc::Pose2d robotPose = GetRobotPose();

    if(str::IsOnRed()) {
      goal = goal + 180_deg;
    }
    thetaController.SetGoal(goal);

    units::radians_per_second_t thetaSpeed{
        thetaController.Calculate(
            robotPose.Rotation().Radians())};

    swerveDrive.Drive(xVel(), yVel(), thetaSpeed, true);
  }, {this})).Until([override] {
    return override();
  });
}

frc2::CommandPtr SwerveSubsystem::FaceSpeaker(
  std::function<units::meters_per_second_t()> xVel,
  std::function<units::meters_per_second_t()> yVel) {
  return frc2::cmd::Sequence(
  frc2::cmd::RunOnce([this] {
    thetaController.EnableContinuousInput(
        units::radian_t{-std::numbers::pi},
        units::radian_t{std::numbers::pi});
    thetaController.SetTolerance(
        consts::swerve::pathplanning::rotationalPIDTolerance,
        consts::swerve::pathplanning::rotationalVelPIDTolerance);
  }, {this}),
  frc2::cmd::Run([this, xVel, yVel] {
    frc::Translation2d speakerLocation = consts::yearSpecific::speakerLocationCenter;

    frc::Pose2d robotPose = GetRobotPose();
    frc::Translation2d robotPosition = robotPose.Translation();
    frc::Translation2d line1Point{0_m, 5.38_m};
    frc::Rotation2d line1Dir{30_deg};
    frc::Translation2d line2Point{0_m, 5.70_m};
    frc::Rotation2d line2Dir{-30_deg};

    bool isInCenter = false;
    bool isAmpSide = false;
    bool isSourceSide = false;
    if (str::IsOnRed()) {
      speakerLocation = pathplanner::FlippingUtil::flipFieldPosition(speakerLocation);
      line1Point = pathplanner::FlippingUtil::flipFieldPosition(line1Point);
      line2Point = pathplanner::FlippingUtil::flipFieldPosition(line2Point);
      isInCenter = str::math::LineBetweenChecker::IsBetweenLines(robotPosition, line1Point, -line1Dir, line2Point, -line2Dir);
      isAmpSide = str::math::LineBetweenChecker::IsPastLine(robotPosition, line2Point, -line2Dir, true);
      isSourceSide = str::math::LineBetweenChecker::IsPastLine(robotPosition, line1Point, -line1Dir, false);
    }
    else {
      isInCenter = str::math::LineBetweenChecker::IsBetweenLines(robotPosition, line1Point, line1Dir, line2Point, line2Dir);
      isAmpSide = str::math::LineBetweenChecker::IsPastLine(robotPosition, line2Point, line2Dir, true);
      isSourceSide = str::math::LineBetweenChecker::IsPastLine(robotPosition, line1Point, line1Dir, false);
    }

    if(!isInCenter) {
      if(isAmpSide) {
        speakerLocation = consts::yearSpecific::speakerLocationSourceSide;
      } else if (isSourceSide) {
        speakerLocation = consts::yearSpecific::speakerLocationAmpSide;
      }
      if(str::IsOnRed()) {
        speakerLocation = pathplanner::FlippingUtil::flipFieldPosition(speakerLocation);
      }
    }


    frc::Translation2d diff = robotPosition - speakerLocation;

    units::radian_t angleToSpeaker = units::math::atan2(diff.Y(), diff.X());

    angleToSpeaker = angleToSpeaker + 180_deg;

    thetaController.SetGoal(angleToSpeaker);

    units::radians_per_second_t thetaSpeed{
        thetaController.Calculate(
            robotPose.Rotation().Radians())};

    swerveDrive.Drive(xVel(), yVel(), thetaSpeed, true);
  }, {this}));
}

frc2::CommandPtr SwerveSubsystem::PointWheelsToAngle(
    std::function<units::radian_t()> wheelAngle) {
  return frc2::cmd::RunOnce(
             [this, wheelAngle] {
               std::array<frc::SwerveModuleState, 4> states;
               for (auto& state : states) {
                 state.angle = wheelAngle();
                 state.speed = 0_mps;
               }

               swerveDrive.SetModuleStates(states, false);
             },
             {this})
      .WithName("Point Wheels At Angle");
}

frc2::CommandPtr SwerveSubsystem::XPattern() {
  return frc2::cmd::Run(
             [this] {
               std::array<frc::SwerveModuleState, 4> states;

               states[0].angle = 45_deg;
               states[1].angle = -45_deg;
               states[2].angle = -45_deg;
               states[3].angle = 45_deg;

               swerveDrive.SetModuleStates(states, true);
             },
             {this})
      .WithName("X Pattern");
}

void SwerveSubsystem::SetupPathplanner() {
  ppControllers = std::make_shared<pathplanner::PPHolonomicDriveController>(
      pathplanner::PIDConstants{consts::swerve::pathplanning::POSE_P,
                                consts::swerve::pathplanning::POSE_I,
                                consts::swerve::pathplanning::POSE_D},
      pathplanner::PIDConstants{consts::swerve::pathplanning::ROTATION_P,
                                consts::swerve::pathplanning::ROTATION_I,
                                consts::swerve::pathplanning::ROTATION_D});

  pathplanner::AutoBuilder::configure(
      [this]() { return GetRobotPose(); },
      [this](frc::Pose2d pose) { swerveDrive.ResetPose(pose); },
      [this]() { return GetRobotRelativeSpeed(); },
      [this](frc::ChassisSpeeds speeds,
             std::vector<pathplanner::DriveFeedforward> ff) {
        frc::ChassisSpeeds speedsToSend =
            frc::ChassisSpeeds::Discretize(speeds, consts::LOOP_PERIOD);

        swerveDrive.SetModuleStates(
            consts::swerve::physical::KINEMATICS.ToSwerveModuleStates(
                speedsToSend),
            true, false);
      },
      ppControllers, consts::swerve::pathplanning::config,
      []() { return str::IsOnRed(); }, this);
}

frc::Translation2d SwerveSubsystem::GetAmpLocation() {
  frc::Translation2d ampToGoTo = consts::yearSpecific::ampLocation;
  if (str::IsOnRed()) {
    ampToGoTo = pathplanner::FlippingUtil::flipFieldPosition(ampToGoTo);
  }
  return ampToGoTo;
}

frc::Translation2d SwerveSubsystem::GetFrontAmpLocation() {
  frc::Translation2d ampToGoTo = consts::yearSpecific::inFrontOfAmpLocation;
  if (str::IsOnRed()) {
    ampToGoTo = pathplanner::FlippingUtil::flipFieldPosition(ampToGoTo);
  }
  return ampToGoTo;
}

frc2::CommandPtr SwerveSubsystem::Stop() {
  return frc2::cmd::RunOnce([this] {
    swerveDrive.Drive(0_fps, 0_fps, 0_rad_per_s, false, false);
  }, {this});
}

bool SwerveSubsystem::IsNearAmp() {
  return GetRobotPose().Translation().Distance(GetAmpLocation()) <
         consts::yearSpecific::closeToAmpDistance;
}

units::meter_t SwerveSubsystem::GetDistanceToSpeaker(const SpeakerSide& side) {
  frc::Translation2d sideToAimAt{};

  if(side == SpeakerSide::AMP_SIDE) {
    sideToAimAt = consts::yearSpecific::speakerLocationAmpSide;
  }
  if(side == SpeakerSide::SOURCE) {
    sideToAimAt = consts::yearSpecific::speakerLocationSourceSide;
  }
  if(side == SpeakerSide::CENTER) {
    sideToAimAt = consts::yearSpecific::speakerLocationCenter;
  }

  if (str::IsOnRed()) {
    sideToAimAt = pathplanner::FlippingUtil::flipFieldPosition(sideToAimAt);
  }
  return GetRobotPose().Translation().Distance(sideToAimAt);
}

frc2::CommandPtr SwerveSubsystem::NoteAssist(
    std::function<units::meters_per_second_t()> xVel,
    std::function<units::meters_per_second_t()> yVel,
    std::function<units::radians_per_second_t()> rotOverride,
    std::function<frc::Pose2d()> notePose) {
  return frc2::cmd::Sequence(
             frc2::cmd::RunOnce(
                 [this, notePose] {
                   frc::Pose2d currentPose = GetRobotPose();
                   frc::ChassisSpeeds currentSpeeds = GetFieldRelativeSpeed();
                   thetaController.Reset(currentPose.Rotation().Radians(),
                                         currentSpeeds.omega);
                   thetaController.EnableContinuousInput(
                       units::radian_t{-std::numbers::pi},
                       units::radian_t{std::numbers::pi});
                   thetaController.SetGoal(notePose().Rotation().Radians());
                   thetaController.SetTolerance(
                       consts::swerve::pathplanning::rotationalPIDTolerance,
                       consts::swerve::pathplanning::rotationalVelPIDTolerance);
                   pidPoseSetpointPub.Set(notePose());
                 },
                 {this})
                 .WithName("NoteAssist Init"),
             frc2::cmd::Run(
                 [this, notePose, xVel, yVel, rotOverride] {
                   frc::Pose2d currentPose = GetRobotPose();

                   frc::Translation2d diff =
                       currentPose.Translation() - notePose().Translation();
                   thetaController.SetGoal(
                       units::math::atan2(diff.Y(), diff.X()));

                   units::radians_per_second_t thetaSpeed{
                       thetaController.Calculate(
                           currentPose.Rotation().Radians())};

                   if (rotOverride() != 0_rad_per_s) {
                     thetaSpeed = rotOverride();
                   }

                   swerveDrive.Drive(xVel(), yVel(), thetaSpeed, true);
                 },
                 {this})
                 .WithName("NoteAssist Run"))
      .WithName("NoteAssist");
}

void SwerveSubsystem::CalculateFoundNotePose(
    std::optional<units::meter_t> distanceToNote,
    std::optional<units::radian_t> angleToNote) {
  frc::Pose3d robotPose = frc::Pose3d{GetRobotPose()};
  if (!angleToNote.has_value() || !distanceToNote.has_value()) {
    latestNotePose = robotPose.ToPose2d();
  } else {
    units::radian_t yaw = angleToNote.value_or(0_deg);
    noteDistPub.Set(distanceToNote.value().value());
    noteYawPub.Set(yaw.convert<units::degrees>().value());
    frc::Transform3d camToNote{distanceToNote.value() * units::math::cos(-yaw),
                               distanceToNote.value() * units::math::sin(-yaw),
                               0_m, frc::Rotation3d{0_rad, 0_rad, 0_rad}};
    frc::Pose3d notePose =
        robotPose.TransformBy(consts::vision::ROBOT_TO_NOTE_CAM)
            .TransformBy(camToNote);
    foundNotePose.Set(
        frc::Pose3d{notePose.X(), notePose.Y(), 0_m, frc::Rotation3d{}});
    latestNotePose = frc::Pose2d{notePose.X(), notePose.Y(), frc::Rotation2d{}};
  }
}

frc2::CommandPtr SwerveSubsystem::ZeroYaw() {
  return frc2::cmd::RunOnce([this] {
    swerveDrive.ZeroYaw();
  });
}

frc::Pose2d SwerveSubsystem::GetFoundNotePose() const {
  return latestNotePose;
}

frc2::CommandPtr SwerveSubsystem::Drive(
    std::function<units::meters_per_second_t()> xVel,
    std::function<units::meters_per_second_t()> yVel,
    std::function<units::radians_per_second_t()> omega, bool fieldRelative,
    bool openLoop) {
  return frc2::cmd::Run(
             [this, xVel, yVel, omega, fieldRelative, openLoop] {
               swerveDrive.Drive(xVel(), yVel(), omega(), fieldRelative,
                                 openLoop);
             },
             {this})
      .WithName("Drive Command");
}

frc2::CommandPtr SwerveSubsystem::TunePosePID(std::function<bool()> isDone) {
  std::string tablePrefix = "SwerveDrive/robotPoseGains/";
  return frc2::cmd::Sequence(
      frc2::cmd::RunOnce(
          [tablePrefix, this] {
            frc::ChassisSpeeds currentSpeeds = GetFieldRelativeSpeed();
            frc::Pose2d currentPose = GetRobotPose();

            frc::SmartDashboard::PutNumber(tablePrefix + "setpointX", 0);
            frc::SmartDashboard::PutNumber(tablePrefix + "setpointY", 0);
            frc::SmartDashboard::PutNumber(
                tablePrefix + "maxSpeed",
                consts::swerve::physical::DRIVE_MAX_SPEED.convert<units::feet_per_second>().value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "maxAccel",
                consts::swerve::physical::DRIVE_MAX_ACCEL.convert<units::feet_per_second_squared>().value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kP",
                consts::swerve::pathplanning::POSE_P.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kI",
                consts::swerve::pathplanning::POSE_I.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kD",
                consts::swerve::pathplanning::POSE_D.value());

            xPoseController.Reset(currentPose.X(), currentSpeeds.vx);
            xPoseController.SetTolerance(
                consts::swerve::pathplanning::translationalPIDTolerance,
                consts::swerve::pathplanning::translationalVelPIDTolerance);

            yPoseController.Reset(currentPose.Y(), currentSpeeds.vy);
            yPoseController.SetTolerance(
                consts::swerve::pathplanning::translationalPIDTolerance,
                consts::swerve::pathplanning::translationalVelPIDTolerance);
          },
          {this}),
      frc2::cmd::Run(
          [this, tablePrefix] {
              frc::Pose2d currentPose = GetRobotPose();
              units::feet_per_second_t newSpeed = units::feet_per_second_t{frc::SmartDashboard::GetNumber(tablePrefix + "maxSpeed", 0)};
              units::feet_per_second_squared_t newAccel = units::feet_per_second_squared_t{frc::SmartDashboard::GetNumber(tablePrefix + "maxAccel", 0)};
              double newP = frc::SmartDashboard::GetNumber(tablePrefix + "kP", 0);
              double newI = frc::SmartDashboard::GetNumber(tablePrefix + "kI", 0);
              double newD = frc::SmartDashboard::GetNumber(tablePrefix + "kD", 0);
              double newGoalX = frc::SmartDashboard::GetNumber(tablePrefix + "setpointX", 0);
              double newGoalY = frc::SmartDashboard::GetNumber(tablePrefix + "setpointY", 0);
              frc::SmartDashboard::PutNumber(tablePrefix + "currentX", currentPose.X().convert<units::feet>().value());
              frc::SmartDashboard::PutNumber(tablePrefix + "currentY", currentPose.Y().convert<units::feet>().value());


              bool vEq = units::essentiallyEqual(xPoseController.GetConstraints().maxVelocity.convert<units::feet_per_second>(), newSpeed, 1e-6);
              bool aEq = units::essentiallyEqual(xPoseController.GetConstraints().maxAcceleration.convert<units::feet_per_second_squared>(), newAccel, 1e-6);
              bool pEq = units::essentiallyEqual(units::scalar_t{xPoseController.GetP()}, units::scalar_t{newP}, 1e-6);
              bool iEq = units::essentiallyEqual(units::scalar_t{xPoseController.GetI()}, units::scalar_t{newI}, 1e-6);
              bool dEq = units::essentiallyEqual(units::scalar_t{xPoseController.GetD()}, units::scalar_t{newD}, 1e-6);
              
              xPoseController.SetGoal(units::foot_t{newGoalX});
              yPoseController.SetGoal(units::foot_t{newGoalY});

              units::meters_per_second_t xSpeed{xPoseController.Calculate(currentPose.X())};
              units::meters_per_second_t ySpeed{yPoseController.Calculate(currentPose.Y())};

              if (!(pEq && iEq && dEq && vEq && aEq)) {
                xPoseController.SetPID(newP, newI, newD);
                xPoseController.SetConstraints(frc::TrapezoidProfile<units::meters>::Constraints{newSpeed, newAccel});
                yPoseController.SetPID(newP, newI, newD);
                yPoseController.SetConstraints(frc::TrapezoidProfile<units::meters>::Constraints{newSpeed, newAccel});
              }

              swerveDrive.Drive(xSpeed, ySpeed, 0_deg_per_s, true, true);
          },
          {this})
          .Until(isDone));
}

frc2::CommandPtr SwerveSubsystem::TuneAnglePID(std::function<bool()> isDone) {
  std::string tablePrefix = "SwerveDrive/robotAngleGains/";
  return frc2::cmd::Sequence(
      frc2::cmd::RunOnce(
          [tablePrefix, this] {
            frc::ChassisSpeeds currentSpeeds = GetFieldRelativeSpeed();
            frc::Pose2d currentPose = GetRobotPose();

            frc::SmartDashboard::PutNumber(tablePrefix + "setpoint", 0);
            frc::SmartDashboard::PutNumber(
                tablePrefix + "maxSpeed",
                consts::swerve::physical::DRIVE_MAX_ROT_SPEED.convert<units::degrees_per_second>().value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "maxAccel",
                consts::swerve::physical::DRIVE_MAX_ROT_ACCEL.convert<units::degrees_per_second_squared>().value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kP",
                consts::swerve::pathplanning::ROTATION_P.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kI",
                consts::swerve::pathplanning::ROTATION_I.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kD",
                consts::swerve::pathplanning::ROTATION_D.value());

            thetaController.Reset(currentPose.Rotation().Radians(), currentSpeeds.omega);
            thetaController.SetTolerance(
                consts::swerve::pathplanning::rotationalPIDTolerance,
                consts::swerve::pathplanning::rotationalVelPIDTolerance);
            thetaController.EnableContinuousInput(
                units::radian_t{-std::numbers::pi},
                units::radian_t{std::numbers::pi});
          },
          {this}),
      frc2::cmd::Run(
          [this, tablePrefix] {
              frc::Pose2d currentPose = GetRobotPose();
              units::degrees_per_second_t newSpeed = units::degrees_per_second_t{frc::SmartDashboard::GetNumber(tablePrefix + "maxSpeed", 0)};
              units::degrees_per_second_squared_t newAccel = units::degrees_per_second_squared_t{frc::SmartDashboard::GetNumber(tablePrefix + "maxAccel", 0)};
              double newP = frc::SmartDashboard::GetNumber(tablePrefix + "kP", 0);
              double newI = frc::SmartDashboard::GetNumber(tablePrefix + "kI", 0);
              double newD = frc::SmartDashboard::GetNumber(tablePrefix + "kD", 0);
              double newGoal = frc::SmartDashboard::GetNumber(tablePrefix + "setpoint", 0);
              frc::SmartDashboard::PutNumber(tablePrefix + "current", currentPose.Rotation().Degrees().value());

              bool vEq = units::essentiallyEqual(thetaController.GetConstraints().maxVelocity.convert<units::degrees_per_second>(), newSpeed, 1e-6);
              bool aEq = units::essentiallyEqual(thetaController.GetConstraints().maxAcceleration.convert<units::degrees_per_second_squared>(), newAccel, 1e-6);
              bool pEq = units::essentiallyEqual(units::scalar_t{thetaController.GetP()}, units::scalar_t{newP}, 1e-6);
              bool iEq = units::essentiallyEqual(units::scalar_t{thetaController.GetI()}, units::scalar_t{newI}, 1e-6);
              bool dEq = units::essentiallyEqual(units::scalar_t{thetaController.GetD()}, units::scalar_t{newD}, 1e-6);
              
              thetaController.SetGoal(units::degree_t{newGoal});

              units::radians_per_second_t thetaSpeed{thetaController.Calculate(currentPose.Rotation().Radians())};

              if (!(pEq && iEq && dEq && vEq && aEq)) {
                thetaController.SetPID(newP, newI, newD);
                thetaController.SetConstraints(frc::TrapezoidProfile<units::radians>::Constraints{newSpeed, newAccel});
              }

              swerveDrive.Drive(0_mps, 0_mps, thetaSpeed, true, true);
          },
          {this})
          .Until(isDone));
}

frc2::CommandPtr SwerveSubsystem::PIDToPose(
    std::function<frc::Pose2d()> goalPose) {
  return frc2::cmd::Sequence(
             frc2::cmd::RunOnce(
                 [this, goalPose] {
                   frc::Pose2d currentPose = GetRobotPose();
                   frc::ChassisSpeeds currentSpeeds = GetFieldRelativeSpeed();
                   xPoseController.Reset(currentPose.Translation().X(),
                                         currentSpeeds.vx);
                   yPoseController.Reset(currentPose.Translation().Y(),
                                         currentSpeeds.vy);
                   thetaController.Reset(currentPose.Rotation().Radians(),
                                         currentSpeeds.omega);
                   thetaController.EnableContinuousInput(
                       units::radian_t{-std::numbers::pi},
                       units::radian_t{std::numbers::pi});
                   xPoseController.SetGoal(goalPose().X());
                   yPoseController.SetGoal(goalPose().Y());
                   thetaController.SetGoal(goalPose().Rotation().Radians());
                   xPoseController.SetTolerance(
                       consts::swerve::pathplanning::translationalPIDTolerance,
                       consts::swerve::pathplanning::
                           translationalVelPIDTolerance);
                   yPoseController.SetTolerance(
                       consts::swerve::pathplanning::translationalPIDTolerance,
                       consts::swerve::pathplanning::
                           translationalVelPIDTolerance);
                   thetaController.SetTolerance(
                       consts::swerve::pathplanning::rotationalPIDTolerance,
                       consts::swerve::pathplanning::rotationalVelPIDTolerance);
                   pidPoseSetpointPub.Set(goalPose());
                 },
                 {this})
                 .WithName("PIDToPose Init"),
             frc2::cmd::Run(
                 [this, goalPose] {
                   frc::Pose2d currentPose = GetRobotPose();

                   xPoseController.SetGoal(goalPose().X());
                   yPoseController.SetGoal(goalPose().Y());
                   thetaController.SetGoal(goalPose().Rotation().Radians());
                   pidPoseSetpointPub.Set(goalPose());

                   units::meters_per_second_t xSpeed{xPoseController.Calculate(
                       currentPose.Translation().X())};
                   units::meters_per_second_t ySpeed{yPoseController.Calculate(
                       currentPose.Translation().Y())};
                   units::radians_per_second_t thetaSpeed{
                       thetaController.Calculate(
                           currentPose.Rotation().Radians())};

                   swerveDrive.Drive(xSpeed, ySpeed, thetaSpeed, true);
                 },
                 {this})
                 .Until([this] {
                   return xPoseController.AtGoal() &&
                          yPoseController.AtGoal() && thetaController.AtGoal();
                 })
                 .WithName("PIDToPose Run"),
             frc2::cmd::Run([this] {
               swerveDrive.Drive(0_mps, 0_mps, 0_deg_per_s, false);
             }).WithName("PIDToPose Stop"))
      .WithName("PIDToPose");
}

frc2::CommandPtr SwerveSubsystem::AlignToAmp() {
  // If we are close enough to the amp, just pid there
  return PIDToPose([this] {
           return frc::Pose2d{GetAmpLocation(), frc::Rotation2d{90_deg}};
         })
      .WithName("AlignToAmp");
}

frc2::CommandPtr SwerveSubsystem::SysIdSteerMk4iQuasistaticTorque(
    frc2::sysid::Direction dir) {
  return frc2::cmd::Sequence(
             frc2::cmd::RunOnce([] { ctre::phoenix6::SignalLogger::Start(); }),
             steerMk4iTorqueSysid.Quasistatic(dir))
      .WithName("Steer Mk4i Quasistatic Torque");
}

frc2::CommandPtr SwerveSubsystem::SysIdSteerMk4iDynamicTorque(
    frc2::sysid::Direction dir) {
  return frc2::cmd::Sequence(
             frc2::cmd::RunOnce([] { ctre::phoenix6::SignalLogger::Start(); }),
             steerMk4iTorqueSysid.Dynamic(dir))
      .WithName("Steer Mk4i Dynamic Torque");
}

frc2::CommandPtr SwerveSubsystem::SysIdSteerMk4nQuasistaticTorque(
    frc2::sysid::Direction dir) {
  return frc2::cmd::Sequence(
             frc2::cmd::RunOnce([] { ctre::phoenix6::SignalLogger::Start(); }),
             steerMk4nTorqueSysid.Quasistatic(dir))
      .WithName("Steer Mk4n Quasistatic Torque");
}

frc2::CommandPtr SwerveSubsystem::SysIdSteerMk4nDynamicTorque(
    frc2::sysid::Direction dir) {
  return frc2::cmd::Sequence(
             frc2::cmd::RunOnce([] { ctre::phoenix6::SignalLogger::Start(); }),
             steerMk4nTorqueSysid.Dynamic(dir))
      .WithName("Steer Mk4n Dynamic Torque");
}

frc2::CommandPtr SwerveSubsystem::SysIdDriveQuasistaticTorque(
    frc2::sysid::Direction dir) {
  return frc2::cmd::Sequence(
             frc2::cmd::RunOnce([] { ctre::phoenix6::SignalLogger::Start(); },
                                {this}),
             frc2::cmd::Parallel(PointWheelsToAngle([] { return 0_rad; }),
                                 frc2::cmd::Wait(1_s)),
             driveTorqueSysid.Quasistatic(dir),
             PointWheelsToAngle([] { return 0_rad; }))
      .WithName("Drive Quasistatic Torque");
}

frc2::CommandPtr SwerveSubsystem::SysIdDriveDynamicTorque(
    frc2::sysid::Direction dir) {
  return frc2::cmd::Sequence(
             frc2::cmd::RunOnce([] { ctre::phoenix6::SignalLogger::Start(); },
                                {this}),
             frc2::cmd::Parallel(PointWheelsToAngle([] { return 0_rad; }),
                                 frc2::cmd::Wait(1_s)),
             driveTorqueSysid.Dynamic(dir),
             PointWheelsToAngle([] { return 0_rad; }))
      .WithName("Drive Dynamic Torque");
}

frc2::CommandPtr SwerveSubsystem::SysIdSteerMk4iQuasistaticVoltage(
    frc2::sysid::Direction dir) {
  return frc2::cmd::Sequence(
             frc2::cmd::RunOnce([] { ctre::phoenix6::SignalLogger::Start(); },
                                {this}),
             steerMk4iVoltageSysid.Quasistatic(dir))
      .WithName("Steer Mk4i Quasistatic Voltage");
}

frc2::CommandPtr SwerveSubsystem::SysIdSteerMk4iDynamicVoltage(
    frc2::sysid::Direction dir) {
  return frc2::cmd::Sequence(
             frc2::cmd::RunOnce([] { ctre::phoenix6::SignalLogger::Start(); },
                                {this}),
             steerMk4iVoltageSysid.Dynamic(dir))
      .WithName("Steer Mk4i Dynamic Voltage");
}

frc2::CommandPtr SwerveSubsystem::SysIdSteerMk4nQuasistaticVoltage(
    frc2::sysid::Direction dir) {
  return frc2::cmd::Sequence(
             frc2::cmd::RunOnce([] { ctre::phoenix6::SignalLogger::Start(); },
                                {this}),
             steerMk4nVoltageSysid.Quasistatic(dir))
      .WithName("Steer Mk4n Quasistatic Voltage");
}

frc2::CommandPtr SwerveSubsystem::SysIdSteerMk4nDynamicVoltage(
    frc2::sysid::Direction dir) {
  return frc2::cmd::Sequence(
             frc2::cmd::RunOnce([] { ctre::phoenix6::SignalLogger::Start(); },
                                {this}),
             steerMk4nVoltageSysid.Dynamic(dir))
      .WithName("Steer Mk4n Dynamic Voltage");
}

frc2::CommandPtr SwerveSubsystem::SysIdDriveQuasistaticVoltage(
    frc2::sysid::Direction dir) {
  return frc2::cmd::Sequence(
             frc2::cmd::RunOnce([] { ctre::phoenix6::SignalLogger::Start(); },
                                {this}),
             frc2::cmd::Parallel(PointWheelsToAngle([] { return 0_rad; }),
                                 frc2::cmd::Wait(1_s)),
             driveVoltageSysid.Quasistatic(dir),
             PointWheelsToAngle([] { return 0_rad; }))
      .WithName("Drive Quasistatic Voltage");
}

frc2::CommandPtr SwerveSubsystem::SysIdDriveDynamicVoltage(
    frc2::sysid::Direction dir) {
  return frc2::cmd::Sequence(
             frc2::cmd::RunOnce([] { ctre::phoenix6::SignalLogger::Start(); },
                                {this}),
             frc2::cmd::Parallel(PointWheelsToAngle([] { return 0_rad; }),
                                 frc2::cmd::Wait(1_s)),
             driveVoltageSysid.Dynamic(dir),
             PointWheelsToAngle([] { return 0_rad; }))
      .WithName("Drive Dynamic Voltage");
}

frc2::CommandPtr SwerveSubsystem::WheelRadius(frc2::sysid::Direction dir) {
  return frc2::cmd::Sequence(
             frc2::cmd::RunOnce(
                 [this] {
                   wheelRadData.lastGyroYaw = swerveDrive.GetYawFromImu();
                   wheelRadData.accumGyroYaw = 0_rad;
                   wheelRadData.startWheelPositions =
                       swerveDrive.GetModuleDriveOutputShaftPositions();
                   wheelRadData.omegaLimiter.Reset(0_rad_per_s);
                   wheelRadData.effectiveWheelRadius = 0_in;
                 },
                 {this}),
             frc2::cmd::RunEnd(
                 [this, dir] {
                   double dirMulti = 1.0;
                   if (dir == frc2::sysid::Direction::kReverse) {
                     dirMulti = -1.0;
                   }
                   units::radian_t currentYaw = swerveDrive.GetYawFromImu();
                   swerveDrive.Drive(0_mps, 0_mps,
                                     wheelRadData.omegaLimiter.Calculate(
                                         1_rad_per_s * dirMulti),
                                     true);
                   wheelRadData.accumGyroYaw +=
                       frc::AngleModulus(currentYaw - wheelRadData.lastGyroYaw);
                   wheelRadData.lastGyroYaw = currentYaw;
                   units::radian_t avgWheelPos = 0.0_rad;
                   std::array<units::radian_t, 4> currentPositions;
                   currentPositions =
                       swerveDrive.GetModuleDriveOutputShaftPositions();
                   for (int i = 0; i < 4; i++) {
                     avgWheelPos +=
                         units::math::abs(currentPositions[i] -
                                          wheelRadData.startWheelPositions[i]);
                   }
                   avgWheelPos /= 4.0;
                   wheelRadData.effectiveWheelRadius =
                       (wheelRadData.accumGyroYaw *
                        consts::swerve::physical::DRIVEBASE_RADIUS) /
                       avgWheelPos;
                 },
                 [this] {
                   swerveDrive.Drive(0_mps, 0_mps, 0_rad_per_s, true);
                   frc::DataLogManager::Log(
                       fmt::format("WHEEL RADIUS: {}\n\n\n\n\n",
                                   wheelRadData.effectiveWheelRadius
                                       .convert<units::inches>()
                                       .value()));
                 },
                 {this}))
      .WithName("Wheel Radius Calculation");
}

frc2::CommandPtr SwerveSubsystem::TuneSteerPID(std::function<bool()> isDone) {
  std::string tablePrefix = "SwerveDrive/steerGains/";
  return frc2::cmd::Sequence(
      frc2::cmd::RunOnce(
          [tablePrefix, this] {
            frc::SmartDashboard::PutNumber(tablePrefix + "setpoint", 0);
            frc::SmartDashboard::PutNumber(
                tablePrefix + "mmCruiseVel",
                consts::swerve::gains::MK4I_STEER_CRUISE_VEL.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "mmKA",
                consts::swerve::gains::MK4I_STEER_MOTION_MAGIC_KA.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "mmKV",
                consts::swerve::gains::MK4I_STEER_MOTION_MAGIC_KV.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kA",
                consts::swerve::gains::MK4I_STEER_KA.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kV",
                consts::swerve::gains::MK4I_STEER_KV.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kS",
                consts::swerve::gains::MK4I_STEER_KS.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kP",
                consts::swerve::gains::MK4I_STEER_KP.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kI",
                consts::swerve::gains::MK4I_STEER_KI.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kD",
                consts::swerve::gains::MK4I_STEER_KD.value());
            frc::SwerveModuleState zeroState{0_mps, frc::Rotation2d{0_rad}};
            swerveDrive.SetModuleStates(
                {zeroState, zeroState, zeroState, zeroState}, true, true);
          },
          {this}),
      frc2::cmd::Run(
          [this, tablePrefix] {
            str::SwerveModuleSteerGains newGains{
                units::turns_per_second_t{frc::SmartDashboard::GetNumber(
                    tablePrefix + "mmCruiseVel", 0)},
                str::gains::radial::turn_volt_ka_unit_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "mmKA", 0)},
                str::gains::radial::turn_volt_kv_unit_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "mmKV", 0)},
                str::gains::radial::turn_amp_ka_unit_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kA", 0)},
                str::gains::radial::turn_amp_kv_unit_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kV", 0)},
                units::ampere_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kS", 0)},
                str::gains::radial::turn_amp_kp_unit_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kP", 0)},
                str::gains::radial::turn_amp_ki_unit_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kI", 0)},
                str::gains::radial::turn_amp_kd_unit_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kD", 0)}};

            if (newGains != swerveDrive.GetSteerGains()) {
              for (int i = 0; i < 4; i++) {
                swerveDrive.SetSteerGains(newGains);
              }
            }

            for (int i = 0; i < 4; i++) {
              frc::SwerveModuleState state{
                  0_mps, frc::Rotation2d{
                             units::degree_t{frc::SmartDashboard::GetNumber(
                                 tablePrefix + "setpoint", 0)}}};
              swerveDrive.SetModuleStates({state, state, state, state}, true, true);
            }
          },
          {this})
          .Until(isDone));
}

frc2::CommandPtr SwerveSubsystem::TuneDrivePID(std::function<bool()> isDone) {
  std::string tablePrefix = "SwerveDrive/driveGains/";
  return frc2::cmd::Sequence(
      frc2::cmd::RunOnce(
          [tablePrefix, this] {
            frc::SmartDashboard::PutNumber(tablePrefix + "setpoint", 0);
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kA", consts::swerve::gains::DRIVE_KA.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kV", consts::swerve::gains::DRIVE_KV.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kS", consts::swerve::gains::DRIVE_KS.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kP", consts::swerve::gains::DRIVE_KP.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kI", consts::swerve::gains::DRIVE_KI.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kD", consts::swerve::gains::DRIVE_KD.value());
            frc::SwerveModuleState zeroState{0_mps, frc::Rotation2d{0_rad}};
            swerveDrive.SetModuleStates(
                {zeroState, zeroState, zeroState, zeroState});
          },
          {this}),
      frc2::cmd::Run(
          [this, tablePrefix] {
            str::SwerveModuleDriveGains newGains{
                str::gains::radial::turn_amp_ka_unit_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kA", 0)},
                str::gains::radial::turn_amp_kv_unit_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kV", 0)},
                units::ampere_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kS", 0)},
                str::gains::radial::turn_amp_kp_unit_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kP", 0)},
                str::gains::radial::turn_amp_ki_unit_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kI", 0)},
                str::gains::radial::turn_amp_kd_unit_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kD", 0)}};

            if (newGains != swerveDrive.GetDriveGains()) {
              for (int i = 0; i < 4; i++) {
                swerveDrive.SetDriveGains(newGains);
              }
            }

            for (int i = 0; i < 4; i++) {
              frc::SwerveModuleState state{
                  units::feet_per_second_t{frc::SmartDashboard::GetNumber(
                      tablePrefix + "setpoint", 0)},
                  frc::Rotation2d{0_deg}};
              swerveDrive.SetModuleStates({state, state, state, state});
            }
          },
          {this})
          .Until(isDone));
}
