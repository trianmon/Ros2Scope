// SPDX-FileCopyrightText: 2026 Georgy Grigoriev (GitHub: trianmon)
// SPDX-License-Identifier: Apache-2.0

#include "ros2scope/workspace_env/workspace_environment.hpp"

#include <cstdlib>
#include <sstream>

namespace ros2scope::workspace_env {

WorkspaceEnvironment::WorkspaceEnvironment()
    : ament_prefix_path_(splitEnvList(std::getenv("AMENT_PREFIX_PATH"))),
      colcon_prefix_path_(splitEnvList(std::getenv("COLCON_PREFIX_PATH"))),
      ld_library_path_(splitEnvList(std::getenv("LD_LIBRARY_PATH"))),
      ros_package_path_(splitEnvList(std::getenv("ROS_PACKAGE_PATH"))) {}

const std::vector<std::string> &WorkspaceEnvironment::amentPrefixPath() const {
  return ament_prefix_path_;
}

const std::vector<std::string> &WorkspaceEnvironment::colconPrefixPath() const {
  return colcon_prefix_path_;
}

const std::vector<std::string> &WorkspaceEnvironment::ldLibraryPath() const {
  return ld_library_path_;
}

const std::vector<std::string> &WorkspaceEnvironment::rosPackagePath() const {
  return ros_package_path_;
}

std::vector<std::string> WorkspaceEnvironment::splitEnvList(const char *value) {
  std::vector<std::string> output;
  if (value == nullptr || value[0] == '\0') {
    return output;
  }

  std::stringstream stream(value);
  std::string token;
  while (std::getline(stream, token, ':')) {
    if (!token.empty()) {
      output.push_back(token);
    }
  }

  return output;
}

}  // namespace ros2scope::workspace_env
