#include "str/struct/SwerveHelperStruct.h"

namespace {
constexpr size_t start = 0;
constexpr size_t one = start + 8;
constexpr size_t two = one + 8;
constexpr size_t three = two + 8;
constexpr size_t four = three + 8;
constexpr size_t five = four + 8;
constexpr size_t six = five + 8;
constexpr size_t seven = six + 8;
constexpr size_t eight = seven + 8;
}  // namespace

using SteerStructType = wpi::Struct<str::SwerveModuleSteerGains>;

str::SwerveModuleSteerGains SteerStructType::Unpack(std::span<const uint8_t> data) {
  return str::SwerveModuleSteerGains{
      units::turns_per_second_t{wpi::UnpackStruct<double, start>(data)},
      str::gains::radial::turn_volt_ka_unit_t{wpi::UnpackStruct<double, one>(data)},
      str::gains::radial::turn_volt_kv_unit_t{wpi::UnpackStruct<double, two>(data)},
      str::gains::radial::turn_amp_ka_unit_t{wpi::UnpackStruct<double, three>(data)},
      str::gains::radial::turn_amp_kv_unit_t{wpi::UnpackStruct<double, four>(data)},
      units::ampere_t{wpi::UnpackStruct<double, five>(data)},
      str::gains::radial::turn_amp_kp_unit_t{wpi::UnpackStruct<double, six>(data)},
      str::gains::radial::turn_amp_ki_unit_t{wpi::UnpackStruct<double, seven>(data)},
      str::gains::radial::turn_amp_kd_unit_t{wpi::UnpackStruct<double, eight>(data)},
  };
}

void SteerStructType::Pack(std::span<uint8_t> data,
                      const str::SwerveModuleSteerGains& value) {
  wpi::PackStruct<start>(data, value.motionMagicCruiseVel.value());
  wpi::PackStruct<one>(data, value.motionMagicExpoKa.value());
  wpi::PackStruct<two>(data, value.motionMagicExpoKv.value());
  wpi::PackStruct<three>(data, value.kA.value());
  wpi::PackStruct<four>(data, value.kV.value());
  wpi::PackStruct<five>(data, value.kS.value());
  wpi::PackStruct<six>(data, value.kP.value());
  wpi::PackStruct<seven>(data, value.kI.value());
  wpi::PackStruct<eight>(data, value.kD.value());
}