// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "str/NoteVisualizer.h"

using namespace str;

NoteVisualizer::NoteVisualizer()  {
    stagedNotesPub.Set(initialNoteLocations);
    launchedNotesPub.Set(launchedNotePoses);
    robotNotePub.Set(robotNote);
}

void NoteVisualizer::LaunchNote(frc::Pose3d currentRobotPose, frc::ChassisSpeeds robotCurrentVelocity, frc::Pose3d noteExitPose, units::meters_per_second_t initialVelocity) {
    NoteVelocity noteVelocity;
    noteVelocity.xVel = robotCurrentVelocity.vx + initialVelocity;
    noteVelocity.yVel = robotCurrentVelocity.vy + initialVelocity;
    noteVelocity.zVel = 0_mps;

    FlyingNote noteToAdd;
    noteToAdd.initialPose = currentRobotPose.RelativeTo(noteExitPose);
    noteToAdd.initialVelocity = noteVelocity;
    noteToAdd.currentPose = noteToAdd.initialPose;
}

void NoteVisualizer::Periodic() {
    UpdateLaunchedNotes();
}

void NoteVisualizer::UpdateLaunchedNotes() {
    int i = 0;
    for(auto& note : launchedNotes) {
        ProjectileMotion(note);
        launchedNotePoses[i] = note.currentPose;
    }
}

void NoteVisualizer::ProjectileMotion(FlyingNote& note) {
    //note.
}

