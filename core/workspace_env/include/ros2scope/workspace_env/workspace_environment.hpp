// SPDX-FileCopyrightText: 2026 Georgy Grigoriev (GitHub: trianmon)
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <string>
#include <vector>

namespace ros2scope::workspace_env {

class WorkspaceEnvironment {
public:
  WorkspaceEnvironment();

  const std::vector<std::string> &amentPrefixPath() const;
  const std::vector<std::string> &colconPrefixPath() const;
  const std::vector<std::string> &ldLibraryPath() const;
  const std::vector<std::string> &rosPackagePath() const;

private:
  static std::vector<std::string> splitEnvList(const char *value);

  std::vector<std::string> ament_prefix_path_;
  std::vector<std::string> colcon_prefix_path_;
  std::vector<std::string> ld_library_path_;
  std::vector<std::string> ros_package_path_;
};

}  // namespace ros2scope::workspace_env
