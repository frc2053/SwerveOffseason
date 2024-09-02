// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

IntakeSubsystem::IntakeSubsystem() {

    ConfigureIntakeMotor(consts::intake::physical::INVERT_MOTOR, consts::intake::physical::INTAKE_RATIO, consts::intake::current_limits::SUPPLY_CURRENT_LIMIT, consts::intake::current_limits::STATOR_CURRENT_LIMIT);
    ConfigureMotorSignals();
    SetName("IntakeSubsystem");
    frc::SmartDashboard::PutData(this);
}

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic() {}

bool IntakeSubsystem::ConfigureIntakeMotor(bool invert,
                            units::scalar_t intakeGearing,
                            units::ampere_t supplyCurrentLimit,
                            units::ampere_t statorCurrentLimit) {
                                return true;
                            }


bool IntakeSubsystem::ConfigureMotorSignals() {
    return true;
}