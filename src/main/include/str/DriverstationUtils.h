#pragma once

#include <frc/Driverstation.h>

namespace str {
  static bool IsOnRed() {
    auto ally = frc::DriverStation::GetAlliance();
    if(ally) {
      return ally.value() == frc::DriverStation::Alliance::kRed;
    }
    return false;
  }
}