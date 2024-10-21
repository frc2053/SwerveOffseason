// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include "constants/Constants.h"

namespace str {
namespace math {

class LineBetweenChecker {
public:
    static bool IsBetweenLines(const frc::Translation2d& objectPos,
                               const frc::Translation2d& line1Point,
                               const frc::Rotation2d& line1Dir,
                               const frc::Translation2d& line2Point,
                               const frc::Rotation2d& line2Dir) {
        // Calculate normal vectors (rotate 90 degrees counterclockwise)
        frc::Rotation2d normal1 = line1Dir + frc::Rotation2d{units::degree_t(90)};
        frc::Rotation2d normal2 = line2Dir + frc::Rotation2d{units::degree_t(90)};
        
        // Calculate signed distances
        double dist1 = CalculateSignedDistance(objectPos, line1Point, normal1);
        double dist2 = CalculateSignedDistance(objectPos, line2Point, normal2);
        
        // Check if signs are opposite
        return (dist1 * dist2 < 0);
    }

    static bool IsPastLine(const frc::Translation2d& objectPos,
                           const frc::Translation2d& linePoint,
                           const frc::Rotation2d& lineDir,
                           bool checkPositiveSide = true) {
        // Calculate normal vector (rotate 90 degrees counterclockwise)
        frc::Rotation2d normal = lineDir + frc::Rotation2d{units::degree_t(90)};
        
        // Calculate signed distance
        double dist = CalculateSignedDistance(objectPos, linePoint, normal);
        
        // Check if the object is on the positive or negative side of the line
        return checkPositiveSide ? (dist > 0) : (dist < 0);
    }

private:
    static double CalculateSignedDistance(const frc::Translation2d& point,
                                          const frc::Translation2d& linePoint,
                                          const frc::Rotation2d& normal) {
        frc::Translation2d diff = point - linePoint;
        return diff.X().to<double>() * normal.Cos() + diff.Y().to<double>() * normal.Sin();
    }
};


static bool IsPointInsideField(const frc::Translation2d& point) {
  return point.X() >= 0_m && point.Y() >= 0_m &&
         point.X() <= consts::yearSpecific::aprilTagLayout.GetFieldLength() &&
         point.Y() <= consts::yearSpecific::aprilTagLayout.GetFieldWidth();
}

static bool IsPoseInsideField(const frc::Pose2d& poseToCheck) {
  units::radian_t currentRotation = poseToCheck.Rotation().Radians();
  units::meter_t ox = (consts::swerve::physical::TOTAL_WIDTH / 2);
  units::meter_t oy = (consts::swerve::physical::TOTAL_LENGTH / 2);

  frc::Translation2d flCorner{
      poseToCheck.X() + (ox * units::math::cos(currentRotation)) -
          (oy * units::math::sin(currentRotation)),
      poseToCheck.Y() + (ox * units::math::sin(currentRotation)) +
          (oy * units::math::cos(currentRotation))};

  frc::Translation2d frCorner{
      poseToCheck.X() + (ox * units::math::cos(currentRotation)) +
          (oy * units::math::sin(currentRotation)),
      poseToCheck.Y() + (ox * units::math::sin(currentRotation)) -
          (oy * units::math::cos(currentRotation))};

  frc::Translation2d blCorner{
      poseToCheck.X() - (ox * units::math::cos(currentRotation)) -
          (oy * units::math::sin(currentRotation)),
      poseToCheck.Y() - (ox * units::math::sin(currentRotation)) +
          (oy * units::math::cos(currentRotation))};

  frc::Translation2d brCorner{
      poseToCheck.X() - (ox * units::math::cos(currentRotation)) +
          (oy * units::math::sin(currentRotation)),
      poseToCheck.Y() - (ox * units::math::sin(currentRotation)) -
          (oy * units::math::cos(currentRotation))};

  return math::IsPointInsideField(flCorner) &&
         math::IsPointInsideField(frCorner) &&
         math::IsPointInsideField(blCorner) &&
         math::IsPointInsideField(brCorner);
}
}  // namespace math
}  // namespace str
