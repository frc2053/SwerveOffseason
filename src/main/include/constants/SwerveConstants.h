#pragma once

#include <units/base.h>
#include <units/velocity.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>

namespace consts {
namespace swerve {
namespace can_ids {
inline constexpr int FL_STEER = 2;
inline constexpr int FL_DRIVE = 3;
inline constexpr int FL_ENC = 4;

inline constexpr int FR_STEER = 5;
inline constexpr int FR_DRIVE = 6;
inline constexpr int FR_ENC = 7;

inline constexpr int BL_STEER = 8;
inline constexpr int BL_DRIVE = 9;
inline constexpr int BL_ENC = 10;

inline constexpr int BR_STEER = 11;
inline constexpr int BR_DRIVE = 12;
inline constexpr int BR_ENC = 13;

inline constexpr int IMU = 14;
}

namespace current_limits {
inline constexpr units::ampere_t SUPPLY_CURRENT_LIMIT = 60_A;
inline constexpr units::ampere_t SLIP_CURRENT_LIMIT = 180_A;
}

namespace physical {

inline constexpr units::turn_t FL_ENC_OFFSET = 0.25_tr;
inline constexpr units::turn_t FR_ENC_OFFSET = 0.25_tr;
inline constexpr units::turn_t BL_ENC_OFFSET = 0.25_tr;
inline constexpr units::turn_t BR_ENC_OFFSET = 0.25_tr;

inline constexpr bool FL_STEER_INVERT = false;
inline constexpr bool FR_STEER_INVERT = false;
inline constexpr bool BL_STEER_INVERT = false;
inline constexpr bool BR_STEER_INVERT = false;

inline constexpr bool FL_DRIVE_INVERT = false;
inline constexpr bool FR_DRIVE_INVERT = false;
inline constexpr bool BL_DRIVE_INVERT = false;
inline constexpr bool BR_DRIVE_INVERT = false;

//SDS MK4i
inline constexpr units::scalar_t STEER_GEARING = (50.0 / 14.0) * (60.0 / 10.0);
//L2 with 16T pinion
inline constexpr units::scalar_t DRIVE_GEARING = (50.0 / 16.0) * (17.0 / 27.0) * (45.0 / 15.0);
inline constexpr units::scalar_t COUPLING_RATIO = (50.0 / 16.0);

inline constexpr frc::DCMotor DRIVE_MOTOR = frc::DCMotor::Falcon500FOC(1);
inline constexpr frc::DCMotor STEER_MOTOR = frc::DCMotor::Falcon500FOC(1);

inline constexpr units::meter_t WHEEL_RADIUS = 2_in;

inline constexpr units::meter_t WHEELBASE_WIDTH = 22.75_in;
inline constexpr units::meter_t WHEELBASE_LENGTH = 22.75_in;

inline const units::meter_t DRIVEBASE_RADIUS{units::math::hypot(WHEELBASE_WIDTH / 2, WHEELBASE_LENGTH / 2)};

inline constexpr std::array<frc::Translation2d, 4> MODULE_LOCATIONS{
    frc::Translation2d{WHEELBASE_LENGTH / 2, WHEELBASE_WIDTH / 2},
    frc::Translation2d{WHEELBASE_LENGTH / 2, -WHEELBASE_WIDTH / 2},
    frc::Translation2d{-WHEELBASE_LENGTH / 2, WHEELBASE_WIDTH / 2},
    frc::Translation2d{-WHEELBASE_LENGTH / 2, -WHEELBASE_WIDTH / 2}};

inline frc::SwerveDriveKinematics<4> KINEMATICS{
    MODULE_LOCATIONS[0], MODULE_LOCATIONS[1], MODULE_LOCATIONS[2],
    MODULE_LOCATIONS[3]};

inline constexpr units::meters_per_second_t DRIVE_MAX_SPEED = ((DRIVE_MOTOR.freeSpeed / 1_rad) / DRIVE_GEARING) * WHEEL_RADIUS;
inline constexpr units::radians_per_second_t DRIVE_MAX_ROT_SPEED = 720_deg_per_s;
}

namespace gains {
inline constexpr units::radians_per_second_t STEER_CRUISE_VEL = physical::STEER_MOTOR.freeSpeed / physical::STEER_GEARING;
inline constexpr str::gains::radial::turn_volt_ka_unit_t STEER_MOTION_MAGIC_KA{0};
inline constexpr str::gains::radial::turn_volt_kv_unit_t STEER_MOTION_MAGIC_KV{0};
inline constexpr str::gains::radial::turn_amp_ka_unit_t STEER_KA{0};
inline constexpr str::gains::radial::turn_amp_kv_unit_t STEER_KV{0};
inline constexpr units::ampere_t STEER_KS{0};
inline constexpr units::volt_t STEER_KS_V{1};
inline constexpr str::gains::radial::turn_amp_kp_unit_t STEER_KP{500};
inline constexpr str::gains::radial::turn_amp_ki_unit_t STEER_KI{0};
inline constexpr str::gains::radial::turn_amp_kd_unit_t STEER_KD{0};

inline constexpr str::gains::radial::turn_amp_ka_unit_t DRIVE_KA{0.31157};
inline constexpr str::gains::radial::turn_amp_kv_unit_t DRIVE_KV{0};
inline constexpr units::ampere_t DRIVE_KS{25.247};
inline constexpr units::volt_t DRIVE_KS_V{1};
inline constexpr str::gains::radial::turn_amp_kp_unit_t DRIVE_KP{3.596};
inline constexpr str::gains::radial::turn_amp_ki_unit_t DRIVE_KI{0};
inline constexpr str::gains::radial::turn_amp_kd_unit_t DRIVE_KD{0};
}

namespace pathplanning {

inline constexpr units::scalar_t POSE_P = 5;
inline constexpr units::scalar_t POSE_I = 0;
inline constexpr units::scalar_t POSE_D = 0;

inline constexpr units::scalar_t ROTATION_P = 5;
inline constexpr units::scalar_t ROTATION_I = 0;
inline constexpr units::scalar_t ROTATION_D = 0;

inline constexpr bool INITIAL_REPLAN = true;
inline constexpr bool DYNAMIC_REPLAN = false;
inline constexpr units::meter_t DYNAMIC_REPLAN_THRESHOLD_TOTAL = 3_ft;
inline constexpr units::meter_t DYNAMIC_REPLAN_THRESHOLD_SPIKE = 1_ft; 

inline const pathplanner::HolonomicPathFollowerConfig PATH_CONFIG{
    pathplanner::PIDConstants{
        POSE_P,
        POSE_I,
        POSE_D
    },
    pathplanner::PIDConstants{
        ROTATION_P,
        ROTATION_I,
        ROTATION_D
    },
    physical::DRIVE_MAX_SPEED,
    physical::DRIVEBASE_RADIUS,
    pathplanner::ReplanningConfig{
        INITIAL_REPLAN,
        DYNAMIC_REPLAN,
        DYNAMIC_REPLAN_THRESHOLD_TOTAL,
        DYNAMIC_REPLAN_THRESHOLD_SPIKE
    }
};
}
}
}