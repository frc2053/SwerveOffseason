// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include <cstring>
#include <regex>

static const char *dev_ = "dev";

namespace photon {
namespace PhotonVersion {
const char *versionString = "UNKNOWN";
const char *buildDate = "UNKNOWN";
const bool isRelease =
    std::strncmp(dev_, versionString, std::strlen(dev_)) != 0;
} // namespace PhotonVersion
} // namespace photon
