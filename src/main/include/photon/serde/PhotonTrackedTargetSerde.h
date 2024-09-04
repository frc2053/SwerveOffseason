/*
 * Copyright (C) Photon Vision.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

// THIS std::FILE WAS AUTO-GENERATED BY ./photon-serde/generate_messages.py. DO
// NOT MODIFY

#include <wpi/SymbolExports.h>

// Include myself
#include "photon/dataflow/structures/Packet.h"
#include "photon/targeting/PhotonTrackedTarget.h"

// Includes for dependant types
#include "photon/targeting/TargetCorner.h"
#include <frc/geometry/Transform3d.h>
#include <stdint.h>
#include <vector>

namespace photon {

template <>
struct WPILIB_DLLEXPORT SerdeType<PhotonTrackedTarget> {
  static constexpr std::string_view GetSchemaHash() {
    return "8fdada56b9162f2e32bd24f0055d7b60";
  }

  static constexpr std::string_view GetSchema() {
    return "float64 yaw;float64 pitch;float64 area;float64 skew;int32 "
           "fiducialId;int32 objDetectId;float32 objDetectConf;Transform3d "
           "bestCameraToTarget;Transform3d altCameraToTarget;float64 "
           "poseAmbiguity;TargetCorner[?] minAreaRectCorners;TargetCorner[?] "
           "detectedCorners;";
  }

  static photon::PhotonTrackedTarget Unpack(photon::Packet& packet);
  static void Pack(photon::Packet& packet,
                   const photon::PhotonTrackedTarget& value);
};

static_assert(photon::PhotonStructSerializable<photon::PhotonTrackedTarget>);

}  // namespace photon
