// SPDX-FileCopyrightText: 2026 Georgy Grigoriev (GitHub: trianmon)
// SPDX-License-Identifier: Apache-2.0

#include "ros2scope/type_introspection/type_resolver.hpp"

#include "ros2scope/workspace_env/workspace_environment.hpp"

#include <filesystem>
#include <string>

#include <rcpputils/shared_library.hpp>

namespace ros2scope::type_introspection {

TypeResolver::TypeResolver(const ros2scope::workspace_env::WorkspaceEnvironment &workspace_env)
    : workspace_env_(workspace_env) {}

bool TypeResolver::ensureTypeSupportLoaded(const std::string &type_name) {
  const std::string package = packageFromType(type_name);
  if (package.empty()) {
    return false;
  }

  if (loaded_libs_.contains(package)) {
    return true;
  }

  const std::string lib_name = "lib" + package + "__rosidl_typesupport_cpp.so";

  for (const auto &prefix : workspace_env_.amentPrefixPath()) {
    std::filesystem::path candidate = std::filesystem::path(prefix) / "lib" / lib_name;
    if (!std::filesystem::exists(candidate)) {
      continue;
    }

    auto library = std::make_shared<rcpputils::SharedLibrary>(candidate.string());
    loaded_libs_[package] = library;
    return true;
  }

  return false;
}

std::string TypeResolver::packageFromType(const std::string &type_name) const {
  const auto slash_pos = type_name.find('/');
  if (slash_pos == std::string::npos) {
    return {};
  }
  return type_name.substr(0, slash_pos);
}

}  // namespace ros2scope::type_introspection
