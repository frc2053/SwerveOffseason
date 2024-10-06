// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "str/DataUtils.h"

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/Filesystem.h>

#include <fstream>
#include <sstream>
#include <string>

using namespace str;

void DataUtils::LogGitInfo() {
  std::string branchFileName =
      frc::filesystem::GetDeployDirectory() + "/" + "branch.txt";

  std::ifstream branchFile(branchFileName);
  if (branchFile.fail()) {
    frc::DataLogManager::Log(fmt::format("Error opening branch file!"));
  }

  std::stringstream branchStream;
  branchStream << branchFile.rdbuf();

  std::string commitFileName =
      frc::filesystem::GetDeployDirectory() + "/" + "commit.txt";
  std::ifstream commitFile(commitFileName);
  if (commitFile.fail()) {
    frc::DataLogManager::Log(fmt::format("Error opening branch file!"));
  }

  std::stringstream commitStream;
  commitStream << commitFile.rdbuf();

  frc::DataLogManager::Log(
      fmt::format("Robot is running code on branch {} with commit {}",
                  branchStream.str(), commitStream.str()));
}
