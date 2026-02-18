// SPDX-FileCopyrightText: 2026 Georgy Grigoriev (GitHub: trianmon)
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <memory>
#include <string>
#include <unordered_map>

namespace rcpputils {
class SharedLibrary;
}

namespace ros2scope::workspace_env {
class WorkspaceEnvironment;
}

namespace ros2scope::type_introspection {

class TypeResolver {
public:
  explicit TypeResolver(const ros2scope::workspace_env::WorkspaceEnvironment &workspace_env);

  bool ensureTypeSupportLoaded(const std::string &type_name);

private:
  std::string packageFromType(const std::string &type_name) const;

  const ros2scope::workspace_env::WorkspaceEnvironment &workspace_env_;
  std::unordered_map<std::string, std::shared_ptr<rcpputils::SharedLibrary>> loaded_libs_;
};

}  // namespace ros2scope::type_introspection
