#pragma once

#include "constants/Constants.h"

namespace str {
  namespace math {
    static bool IsPointInsideField(const frc::Translation2d& point) {
      return point.X() >= 0_m && 
        point.Y() >= 0_m && 
        point.X() <= consts::yearSpecific::aprilTagLayout.GetFieldLength() &&
        point.Y() <= consts::yearSpecific::aprilTagLayout.GetFieldWidth();
    }

    static bool IsPoseInsideField(const frc::Pose2d& poseToCheck) {        
        units::radian_t currentRotation = poseToCheck.Rotation().Radians();
        units::meter_t ox = (consts::swerve::physical::TOTAL_WIDTH / 2);
        units::meter_t oy = (consts::swerve::physical::TOTAL_LENGTH / 2);

        frc::Translation2d flCorner{
          poseToCheck.X() + (ox * units::math::cos(currentRotation)) - (oy * units::math::sin(currentRotation)),
          poseToCheck.Y() + (ox * units::math::sin(currentRotation)) + (oy * units::math::cos(currentRotation))
        };

        frc::Translation2d frCorner{
          poseToCheck.X() + (ox * units::math::cos(currentRotation)) + (oy * units::math::sin(currentRotation)),
          poseToCheck.Y() + (ox * units::math::sin(currentRotation)) - (oy * units::math::cos(currentRotation))
        };

        frc::Translation2d blCorner{
          poseToCheck.X() - (ox * units::math::cos(currentRotation)) - (oy * units::math::sin(currentRotation)),
          poseToCheck.Y() - (ox * units::math::sin(currentRotation)) + (oy * units::math::cos(currentRotation))
        };

        frc::Translation2d brCorner{
          poseToCheck.X() - (ox * units::math::cos(currentRotation)) + (oy * units::math::sin(currentRotation)),
          poseToCheck.Y() - (ox * units::math::sin(currentRotation)) - (oy * units::math::cos(currentRotation))
        };

        return math::IsPointInsideField(flCorner) && math::IsPointInsideField(frCorner) && math::IsPointInsideField(blCorner) && math::IsPointInsideField(brCorner);
    }
  }
}