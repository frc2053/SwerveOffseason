// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "str/NoteVisualizer.h"
#include <frc/Timer.h>

using namespace str;

NoteVisualizer::NoteVisualizer()  {
    stagedNotesPub.Set(initialNoteLocations);
    launchedNotesPub.Set(launchedNotePoses);
    robotNotePub.Set(robotNote);
}

void NoteVisualizer::LaunchNote(frc::Pose3d currentRobotPose, frc::ChassisSpeeds robotCurrentVelocity, frc::Transform3d noteExitPose, units::meters_per_second_t initialVelocity) {
    NoteVelocity noteVelocity;
    noteVelocity.xVel = 0_mps;
    noteVelocity.yVel = robotCurrentVelocity.vy + units::math::sin(noteExitPose.Rotation().Y()) * initialVelocity;
    noteVelocity.zVel = robotCurrentVelocity.vx + units::math::cos(noteExitPose.Rotation().X()) * initialVelocity;

    FlyingNote noteToAdd;
    noteToAdd.initialPose = currentRobotPose.TransformBy(noteExitPose);
    noteToAdd.initialVelocity = noteVelocity;
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
}

void NoteVisualizer::UpdateLaunchedNotes(units::second_t loopTime) {
    int i = 0;
    for(auto& note : launchedNotes) {
        ProjectileMotion(note, loopTime);
        launchedNotePoses[i] = note.currentPose;
        i++;
    }
}

void NoteVisualizer::ProjectileMotion(FlyingNote& note, units::second_t loopTime) {
    note.currentPose = note.currentPose.TransformBy(
        frc::Transform3d{
            note.initialVelocity.xVel * loopTime, 
            note.initialVelocity.yVel * loopTime, 
            note.initialVelocity.zVel * loopTime, 
            frc::Rotation3d{}
        }
    );
}

