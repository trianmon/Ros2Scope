// SPDX-FileCopyrightText: 2026 Georgy Grigoriev (GitHub: trianmon)
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>

namespace ros2scope::plugin_api {

class CoreContext;

inline constexpr std::uint32_t kPluginApiVersion = 1;

class IPlugin {
public:
  virtual const char *name() const = 0;
  virtual const char *version() const = 0;

  virtual void onLoad(CoreContext *core_context) = 0;
  virtual void onUnload() = 0;

  virtual void onUpdate(double dt) = 0;
  virtual void onDraw() = 0;

  virtual ~IPlugin() = default;
};

using CreatePluginFn = IPlugin *(*)();
using GetApiVersionFn = std::uint32_t (*)();

}  // namespace ros2scope::plugin_api
