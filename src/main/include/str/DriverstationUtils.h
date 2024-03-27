#pragma once

#include <frc/Driverstation.h>
#include <functional>

namespace str {

  static bool IsOnRed() {
    auto ally = frc::DriverStation::GetAlliance();
    if(ally) {
      return ally.value() == frc::DriverStation::Alliance::kRed;
    }
    return false;
  }

  template<typename T>
  static T NegateIfRed(T input) {
    return IsOnRed() ? input * -1 : input;
  };
}