// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/system/plant/DCMotor.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <units/base.h>
#include <units/velocity.h>

#include "str/Gains.h"

namespace consts {
namespace swerve {
namespace can_ids {
inline constexpr int FL_DRIVE = 2;
inline constexpr int FL_STEER = 3;
inline constexpr int FL_ENC = 4;

inline constexpr int FR_DRIVE = 5;
inline constexpr int FR_STEER = 6;
inline constexpr int FR_ENC = 7;

inline constexpr int BL_DRIVE = 8;
inline constexpr int BL_STEER = 9;
inline constexpr int BL_ENC = 10;

inline constexpr int BR_DRIVE = 11;
inline constexpr int BR_STEER = 12;
inline constexpr int BR_ENC = 13;

inline constexpr int IMU = 14;
} // namespace can_ids

namespace current_limits {
inline constexpr units::ampere_t STEER_TORQUE_CURRENT_LIMIT = 40_A;
inline constexpr units::ampere_t SUPPLY_CURRENT_LIMIT = 60_A;
inline constexpr units::ampere_t SLIP_CURRENT_LIMIT = 180_A;
} // namespace current_limits

namespace physical {

#ifdef __FRC_ROBORIO__
inline constexpr units::radian_t IMU_ROLL_OFFSET = -0.4168_deg;
inline constexpr units::radian_t IMU_PITCH_OFFSET = -89.879_deg;
inline constexpr units::radian_t IMU_YAW_OFFSET = 89.9885_deg;

inline constexpr units::turn_t FL_ENC_OFFSET = 0.008789_tr;
inline constexpr units::turn_t FR_ENC_OFFSET = -0.303223_tr;
inline constexpr units::turn_t BL_ENC_OFFSET = -0.141357_tr;
inline constexpr units::turn_t BR_ENC_OFFSET = 0.214111_tr;

inline constexpr bool FL_STEER_INVERT = true;
inline constexpr bool FR_STEER_INVERT = true;
inline constexpr bool BL_STEER_INVERT = true;
inline constexpr bool BR_STEER_INVERT = true;

inline constexpr bool FL_DRIVE_INVERT = false;
inline constexpr bool FR_DRIVE_INVERT = true;
inline constexpr bool BL_DRIVE_INVERT = false;
inline constexpr bool BR_DRIVE_INVERT = true;
#else
inline constexpr units::radian_t IMU_ROLL_OFFSET = 0_deg;
inline constexpr units::radian_t IMU_PITCH_OFFSET = 0_deg;
inline constexpr units::radian_t IMU_YAW_OFFSET = 0_deg;

inline constexpr units::turn_t FL_ENC_OFFSET = .25_tr;
inline constexpr units::turn_t FR_ENC_OFFSET = .25_tr;
inline constexpr units::turn_t BL_ENC_OFFSET = .25_tr;
inline constexpr units::turn_t BR_ENC_OFFSET = .25_tr;

inline constexpr bool FL_STEER_INVERT = false;
inline constexpr bool FR_STEER_INVERT = false;
inline constexpr bool BL_STEER_INVERT = false;
inline constexpr bool BR_STEER_INVERT = false;

inline constexpr bool FL_DRIVE_INVERT = false;
inline constexpr bool FR_DRIVE_INVERT = false;
inline constexpr bool BL_DRIVE_INVERT = false;
inline constexpr bool BR_DRIVE_INVERT = false;
#endif


inline constexpr units::scalar_t STEER_GEARING_MK4I =
    (50.0 / 14.0) * (60.0 / 10.0);
inline constexpr units::scalar_t STEER_GEARING_MK4N =
    (50.0 / 16.0) * (60.0 / 10.0);
// L3 with 16T pinion
inline constexpr units::scalar_t DRIVE_GEARING =
    (50.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0);
inline constexpr units::scalar_t COUPLING_RATIO = (50.0 / 16.0);

inline constexpr frc::DCMotor DRIVE_MOTOR = frc::DCMotor::Falcon500FOC(1);
inline constexpr frc::DCMotor STEER_MOTOR = frc::DCMotor::Falcon500FOC(1);

inline constexpr units::meter_t WHEEL_RADIUS = 2_in;

// Distance between center of drive wheels
inline constexpr units::meter_t WHEELBASE_WIDTH = 21.75_in;
inline constexpr units::meter_t WHEELBASE_LENGTH = 15.75_in;

// Total outside frame size
inline constexpr units::meter_t DRIVEBASE_WIDTH = 27.440000_in;
inline constexpr units::meter_t DRIVEBASE_LENGTH = 27_in;

// 3/4 in plywood + 2.5 in diameter pool noodles + 1/8 slop
inline constexpr units::meter_t BUMPER_THICKNESS = .75_in + 2.5_in + .125_in;

// Total size including bumpers
inline constexpr units::meter_t TOTAL_WIDTH =
    DRIVEBASE_WIDTH + (2 * BUMPER_THICKNESS);
inline constexpr units::meter_t TOTAL_LENGTH =
    DRIVEBASE_LENGTH + (2 * BUMPER_THICKNESS);

inline const units::meter_t DRIVEBASE_RADIUS{
    units::math::hypot(WHEELBASE_WIDTH / 2, WHEELBASE_LENGTH / 2)};

inline constexpr std::array<frc::Translation2d, 4> MODULE_LOCATIONS{
    frc::Translation2d{WHEELBASE_LENGTH / 2, WHEELBASE_WIDTH / 2},
    frc::Translation2d{WHEELBASE_LENGTH / 2, -WHEELBASE_WIDTH / 2},
    frc::Translation2d{-WHEELBASE_LENGTH / 2, WHEELBASE_WIDTH / 2},
    frc::Translation2d{-WHEELBASE_LENGTH / 2, -WHEELBASE_WIDTH / 2}};

inline frc::SwerveDriveKinematics<4> KINEMATICS{
    MODULE_LOCATIONS[0], MODULE_LOCATIONS[1], MODULE_LOCATIONS[2],
    MODULE_LOCATIONS[3]};

inline constexpr units::meters_per_second_t DRIVE_MAX_SPEED =
    ((DRIVE_MOTOR.freeSpeed / 1_rad) / DRIVE_GEARING) * WHEEL_RADIUS;
inline constexpr units::radians_per_second_t DRIVE_MAX_ROT_SPEED =
    540_deg_per_s;
inline constexpr units::meters_per_second_squared_t DRIVE_MAX_ACCEL =
    2000_fps_sq;
inline constexpr units::radians_per_second_squared_t DRIVE_MAX_ROT_ACCEL =
    720_deg_per_s_sq;
} // namespace physical

namespace gains {
inline constexpr units::radians_per_second_t MK4I_STEER_CRUISE_VEL =
    physical::STEER_MOTOR.freeSpeed / physical::STEER_GEARING_MK4I;
inline constexpr str::gains::radial::turn_volt_ka_unit_t
    MK4I_STEER_MOTION_MAGIC_KA{.1};
inline constexpr str::gains::radial::turn_volt_kv_unit_t
    MK4I_STEER_MOTION_MAGIC_KV{.12 * physical::STEER_GEARING_MK4I.value()};
inline constexpr str::gains::radial::turn_amp_ka_unit_t MK4I_STEER_KA{0};
inline constexpr str::gains::radial::turn_amp_kv_unit_t MK4I_STEER_KV{0};
inline constexpr units::ampere_t MK4I_STEER_KS{0};
inline constexpr units::volt_t MK4I_STEER_KS_V{.1};
inline constexpr str::gains::radial::turn_amp_kp_unit_t MK4I_STEER_KP{300};
inline constexpr str::gains::radial::turn_amp_ki_unit_t MK4I_STEER_KI{0};
inline constexpr str::gains::radial::turn_amp_kd_unit_t MK4I_STEER_KD{20.654};

inline constexpr units::radians_per_second_t MK4N_STEER_CRUISE_VEL =
    physical::STEER_MOTOR.freeSpeed / physical::STEER_GEARING_MK4N;
inline constexpr str::gains::radial::turn_volt_ka_unit_t
    MK4N_STEER_MOTION_MAGIC_KA{.1};
inline constexpr str::gains::radial::turn_volt_kv_unit_t
    MK4N_STEER_MOTION_MAGIC_KV{.12 * physical::STEER_GEARING_MK4N.value()};
inline constexpr str::gains::radial::turn_amp_ka_unit_t MK4N_STEER_KA{.5};
inline constexpr str::gains::radial::turn_amp_kv_unit_t MK4N_STEER_KV{0};
inline constexpr units::ampere_t MK4N_STEER_KS{19.018};
inline constexpr units::volt_t MK4N_STEER_KS_V{.1};
inline constexpr str::gains::radial::turn_amp_kp_unit_t MK4N_STEER_KP{1000};
inline constexpr str::gains::radial::turn_amp_ki_unit_t MK4N_STEER_KI{0};
inline constexpr str::gains::radial::turn_amp_kd_unit_t MK4N_STEER_KD{50};

inline constexpr str::gains::radial::turn_amp_ka_unit_t DRIVE_KA{51.54};
inline constexpr str::gains::radial::turn_amp_kv_unit_t DRIVE_KV{0};
inline constexpr units::ampere_t DRIVE_KS{25.247};
inline constexpr units::volt_t DRIVE_KS_V{1};
inline constexpr str::gains::radial::turn_amp_kp_unit_t DRIVE_KP{9};
inline constexpr str::gains::radial::turn_amp_ki_unit_t DRIVE_KI{0};
inline constexpr str::gains::radial::turn_amp_kd_unit_t DRIVE_KD{0};
} // namespace gains

namespace pathplanning {

inline constexpr units::scalar_t POSE_P = 5;
inline constexpr units::scalar_t POSE_I = 0;
inline constexpr units::scalar_t POSE_D = 0;

inline constexpr units::scalar_t ROTATION_P = 10;
inline constexpr units::scalar_t ROTATION_I = 0;
inline constexpr units::scalar_t ROTATION_D = 0;

// Choreo paths don't support replanning, so just disable me
inline constexpr bool INITIAL_REPLAN = false;
inline constexpr bool DYNAMIC_REPLAN = false;
inline constexpr units::meter_t DYNAMIC_REPLAN_THRESHOLD_TOTAL = 3_ft;
inline constexpr units::meter_t DYNAMIC_REPLAN_THRESHOLD_SPIKE = 1_ft;

inline const pathplanner::HolonomicPathFollowerConfig PATH_CONFIG{
    pathplanner::PIDConstants{POSE_P, POSE_I, POSE_D},
    pathplanner::PIDConstants{ROTATION_P, ROTATION_I, ROTATION_D},
    physical::DRIVE_MAX_SPEED, physical::DRIVEBASE_RADIUS,
    pathplanner::ReplanningConfig{INITIAL_REPLAN, DYNAMIC_REPLAN,
                                  DYNAMIC_REPLAN_THRESHOLD_TOTAL,
                                  DYNAMIC_REPLAN_THRESHOLD_SPIKE}};

inline constexpr pathplanner::PathConstraints constraints{
    physical::DRIVE_MAX_SPEED, physical::DRIVE_MAX_ACCEL,
    physical::DRIVE_MAX_ROT_SPEED, physical::DRIVE_MAX_ROT_ACCEL};

inline constexpr units::meter_t translationalPIDTolerance = .5_in;
inline constexpr units::meters_per_second_t translationalVelPIDTolerance =
    1_fps;
inline constexpr units::radian_t rotationalPIDTolerance = 1_deg;
inline constexpr units::radians_per_second_t rotationalVelPIDTolerance =
    10_deg_per_s;
inline constexpr units::meters_per_second_t translationalVelPIDDeadband =
    0.5_fps;
inline constexpr units::radians_per_second_t rotationalVelPIDDeadband =
    5_deg_per_s;
} // namespace pathplanning
} // namespace swerve
} // namespace consts
