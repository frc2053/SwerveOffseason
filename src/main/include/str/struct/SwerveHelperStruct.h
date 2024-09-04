#pragma once

#include <wpi/struct/Struct.h>

#include "str/SwerveModuleHelpers.h"

template <>
struct wpi::Struct<str::SwerveModuleSteerGains> {
  static constexpr std::string_view GetTypeName() { return "SwerveModuleSteerGains"; }
  static constexpr size_t GetSize() { return 72; }
  static constexpr std::string_view GetSchema() { return "double motionMagicCruiseVel;double motionMagicExpoKa;double motionMagicExpoKv;double kA;double kV;double kS;double kP;double kI;double kD;"; }

  static str::SwerveModuleSteerGains Unpack(std::span<const uint8_t> data);
  static void Pack(std::span<uint8_t> data, const str::SwerveModuleSteerGains& value);
};

static_assert(wpi::StructSerializable<str::SwerveModuleSteerGains>);

template <>
struct wpi::Struct<str::SwerveModuleDriveGains> {
  static constexpr std::string_view GetTypeName() { return "SwerveModuleDriveGains"; }
  static constexpr size_t GetSize() { return 48; }
  static constexpr std::string_view GetSchema() { return "double kA;double kV;double kS;double kP;double kI;double kD;"; }

  static str::SwerveModuleDriveGains Unpack(std::span<const uint8_t> data);
  static void Pack(std::span<uint8_t> data, const str::SwerveModuleDriveGains& value);
};

static_assert(wpi::StructSerializable<str::SwerveModuleDriveGains>);