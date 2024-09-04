#include <string.h>
#include <regex>

static const char* dev_ = "dev";

namespace photon {
  namespace PhotonVersion {
    const char* versionString = "UNKNOWN";
    const char* buildDate = "UNKNOWN";
    const bool isRelease = strncmp(dev_, versionString, strlen(dev_)) != 0;
  }
}