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

// THIS FILE WAS AUTO-GENERATED BY ./photon-serde/generate_messages.py. DO NOT MODIFY

#include <wpi/SymbolExports.h>

// Include myself
#include "photon/dataflow/structures/Packet.h"
#include "photon/targeting/PhotonPipelineResult.h"

// Includes for dependant types
#include "photon/targeting/MultiTargetPNPResult.h"
#include "photon/targeting/PhotonPipelineMetadata.h"
#include "photon/targeting/PhotonTrackedTarget.h"
#include <optional>
#include <stdint.h>
#include <vector>


namespace photon {

template <>
struct WPILIB_DLLEXPORT SerdeType<PhotonPipelineResult> {
  static constexpr std::string_view GetSchemaHash() {
    return "5eeaa293d0c69aea90eaddea786a2b3b";
  }

  static constexpr std::string_view GetSchema() {
    return "PhotonPipelineMetadata:626e70461cbdb274fb43ead09c255f4e metadata;PhotonTrackedTarget:cc6dbb5c5c1e0fa808108019b20863f1 targets[?];optional MultiTargetPNPResult:541096947e9f3ca2d3f425ff7b04aa7b multitagResult;";
  }

  static photon::PhotonPipelineResult Unpack(photon::Packet& packet);
  static void Pack(photon::Packet& packet, const photon::PhotonPipelineResult& value);
};

static_assert(photon::PhotonStructSerializable<photon::PhotonPipelineResult>);

} // namespace photon