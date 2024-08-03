// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "str/NoteVisualizer.h"

#include <frc/Timer.h>
#include <units/acceleration.h>

using namespace str;

NoteVisualizer::NoteVisualizer() {
  stagedNotesPub.Set(initialNoteLocations);
  launchedNotesPub.Set(launchedNotePoses);
  robotNotePub.Set(robotNote);
}

void NoteVisualizer::LaunchNote(frc::Pose3d currentRobotPose,
                                frc::ChassisSpeeds robotCurrentVelocity,
                                frc::Transform3d noteExitPose,
                                units::meters_per_second_t initialVelocity) {
  NoteVelocity noteVelocity;
  noteVelocity.xVel =
      robotCurrentVelocity.vx +
      (units::math::cos(-noteExitPose.Rotation().Y()) * initialVelocity);
  noteVelocity.yVel = robotCurrentVelocity.vy;
  noteVelocity.zVel =
      units::math::sin(-noteExitPose.Rotation().Y()) * initialVelocity;

  frc::Translation3d noteRotatedVel = frc::Translation3d{
      Eigen::Vector3d{
          noteVelocity.xVel.value(), noteVelocity.yVel.value(),
          noteVelocity.zVel.value()}}.RotateBy(currentRobotPose.Rotation());
  noteVelocity.xVel = units::meters_per_second_t{noteRotatedVel.X().value()};
  noteVelocity.yVel = units::meters_per_second_t{noteRotatedVel.Y().value()};
  noteVelocity.zVel = units::meters_per_second_t{noteRotatedVel.Z().value()};

  FlyingNote noteToAdd;
  noteToAdd.initialPose = currentRobotPose.TransformBy(noteExitPose);
  noteToAdd.initialVelocity = noteVelocity;
  noteToAdd.currentVelocity = noteVelocity;
  noteToAdd.currentPose = noteToAdd.initialPose;

  launchedNotes.emplace_back(noteToAdd);
  launchedNotePoses.emplace_back(noteToAdd.currentPose);
}

void NoteVisualizer::Periodic() {
  units::second_t now = frc::Timer::GetFPGATimestamp();
  units::second_t loopTime = now - lastLoopTime;

  UpdateLaunchedNotes(loopTime);

  stagedNotesPub.Set(initialNoteLocations);
  launchedNotesPub.Set(launchedNotePoses);
  robotNotePub.Set(robotNote);

  lastLoopTime = now;
}

void NoteVisualizer::UpdateLaunchedNotes(units::second_t loopTime) {
  int i = 0;
  for (auto &note : launchedNotes) {
    ProjectileMotion(note, loopTime);
    launchedNotePoses[i] = note.currentPose;
    i++;
  }
}

void NoteVisualizer::ProjectileMotion(FlyingNote &note,
                                      units::second_t loopTime) {
  note.currentVelocity.zVel =
      note.currentVelocity.zVel - (9.81_mps_sq * loopTime);

  frc::Pose3d newPose{
      frc::Translation3d{
          note.currentPose.X() + (note.currentVelocity.xVel * loopTime),
          note.currentPose.Y() + (note.currentVelocity.yVel * loopTime),
          note.currentPose.Z() + (note.currentVelocity.zVel * loopTime),
      },
      note.currentPose.Rotation()};

  if (newPose.Z() <= 1_in) {
    frc::Pose3d groundPose{newPose.X(), newPose.Y(), 1_in, frc::Rotation3d{}};
    newPose = groundPose;
    note.currentVelocity.xVel = 0_mps;
    note.currentVelocity.yVel = 0_mps;
    note.currentVelocity.zVel = 0_mps;
  }

  note.currentPose = newPose;
}
