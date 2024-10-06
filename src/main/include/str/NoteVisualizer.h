// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/geometry/Pose3d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>
#include <units/velocity.h>

#include <memory>
#include <vector>

#include "constants/Constants.h"

namespace str {

struct NoteVelocity {
  units::meters_per_second_t xVel;
  units::meters_per_second_t yVel;
  units::meters_per_second_t zVel;
};

struct FlyingNote {
  frc::Pose3d initialPose{};
  NoteVelocity initialVelocity{};
  NoteVelocity currentVelocity{};
  frc::Pose3d currentPose{};
  bool shouldClean{false};
};

class NoteVisualizer {
 public:
  NoteVisualizer();
  void Periodic();
  void LaunchNote(frc::Pose3d currentRobotPose,
                  frc::ChassisSpeeds robotCurrentVelocity,
                  frc::Transform3d noteExitPose,
                  units::meters_per_second_t initialVelocity);
  void DisplayRobotNote(bool hasNote, const frc::Pose2d& robotPosition);

 private:
  void UpdateLaunchedNotes(units::second_t loopTime);
  void ProjectileMotion(FlyingNote& note, units::second_t loopTime);
  void CleanUp();

  units::second_t lastLoopTime;

  static constexpr units::meter_t NOTE_VERTICAL_SPACING = 57_in;
  static constexpr units::meter_t NOTE_MIDDLE_SPACING = 66_in;
  static constexpr units::meter_t NOTE_HORIZONTAL_SPACING = 114_in;
  static constexpr units::meter_t NOTE_GROUND_HEIGHT = 1_in;
  const units::meter_t ALLY_NOTE_VERTICAL_OFFSET =
      consts::yearSpecific::aprilTagLayout.GetFieldWidth() / 2;
  const units::meter_t MIDDLE_NOTE_VERTICAL_OFFSET =
      consts::yearSpecific::aprilTagLayout.GetFieldWidth() - 29.64_in;
  const units::meter_t NOTE_RED_HORIZONTAL_OFFSET =
      consts::yearSpecific::aprilTagLayout.GetFieldLength();
  const units::meter_t NOTE_MIDDLE_OFFSET =
      consts::yearSpecific::aprilTagLayout.GetFieldLength() / 2;

  std::array<frc::Pose3d, 11> initialNoteLocations{
      // BLUE SIDE
      frc::Pose3d{
          frc::Translation3d{NOTE_HORIZONTAL_SPACING, ALLY_NOTE_VERTICAL_OFFSET,
                             NOTE_GROUND_HEIGHT},
          frc::Rotation3d{}},
      frc::Pose3d{frc::Translation3d{
                      NOTE_HORIZONTAL_SPACING,
                      ALLY_NOTE_VERTICAL_OFFSET + (NOTE_VERTICAL_SPACING * 1),
                      NOTE_GROUND_HEIGHT},
                  frc::Rotation3d{}},
      frc::Pose3d{frc::Translation3d{
                      NOTE_HORIZONTAL_SPACING,
                      ALLY_NOTE_VERTICAL_OFFSET + (NOTE_VERTICAL_SPACING * 2),
                      NOTE_GROUND_HEIGHT},
                  frc::Rotation3d{}},
      // MIDDLE
      frc::Pose3d{
          frc::Translation3d{NOTE_MIDDLE_OFFSET, MIDDLE_NOTE_VERTICAL_OFFSET,
                             NOTE_GROUND_HEIGHT},
          frc::Rotation3d{}},
      frc::Pose3d{frc::Translation3d{
                      NOTE_MIDDLE_OFFSET,
                      MIDDLE_NOTE_VERTICAL_OFFSET - (NOTE_MIDDLE_SPACING * 1),
                      NOTE_GROUND_HEIGHT},
                  frc::Rotation3d{}},
      frc::Pose3d{frc::Translation3d{
                      NOTE_MIDDLE_OFFSET,
                      MIDDLE_NOTE_VERTICAL_OFFSET - (NOTE_MIDDLE_SPACING * 2),
                      NOTE_GROUND_HEIGHT},
                  frc::Rotation3d{}},
      frc::Pose3d{frc::Translation3d{
                      NOTE_MIDDLE_OFFSET,
                      MIDDLE_NOTE_VERTICAL_OFFSET - (NOTE_MIDDLE_SPACING * 3),
                      NOTE_GROUND_HEIGHT},
                  frc::Rotation3d{}},
      frc::Pose3d{frc::Translation3d{
                      NOTE_MIDDLE_OFFSET,
                      MIDDLE_NOTE_VERTICAL_OFFSET - (NOTE_MIDDLE_SPACING * 4),
                      NOTE_GROUND_HEIGHT},
                  frc::Rotation3d{}},
      // RED SIDE
      frc::Pose3d{frc::Translation3d{
                      NOTE_RED_HORIZONTAL_OFFSET - NOTE_HORIZONTAL_SPACING,
                      ALLY_NOTE_VERTICAL_OFFSET, NOTE_GROUND_HEIGHT},
                  frc::Rotation3d{}},
      frc::Pose3d{frc::Translation3d{
                      NOTE_RED_HORIZONTAL_OFFSET - NOTE_HORIZONTAL_SPACING,
                      ALLY_NOTE_VERTICAL_OFFSET + (NOTE_VERTICAL_SPACING * 1),
                      NOTE_GROUND_HEIGHT},
                  frc::Rotation3d{}},
      frc::Pose3d{frc::Translation3d{
                      NOTE_RED_HORIZONTAL_OFFSET - NOTE_HORIZONTAL_SPACING,
                      ALLY_NOTE_VERTICAL_OFFSET + (NOTE_VERTICAL_SPACING * 2),
                      NOTE_GROUND_HEIGHT},
                  frc::Rotation3d{}}};

  std::vector<FlyingNote> launchedNotes{};
  std::vector<frc::Pose3d> launchedNotePoses{};
  frc::Pose3d robotNote{};

  std::shared_ptr<nt::NetworkTable> nt{
      nt::NetworkTableInstance::GetDefault().GetTable("NoteVisualizer")};
  nt::StructArrayPublisher<frc::Pose3d> stagedNotesPub{
      nt->GetStructArrayTopic<frc::Pose3d>("StagedNotes").Publish()};
  nt::StructArrayPublisher<frc::Pose3d> launchedNotesPub{
      nt->GetStructArrayTopic<frc::Pose3d>("LaunchedNotes").Publish()};
  nt::StructPublisher<frc::Pose3d> robotNotePub{
      nt->GetStructTopic<frc::Pose3d>("RobotNote").Publish()};
};
}  // namespace str
