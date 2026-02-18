// SPDX-FileCopyrightText: 2026 Georgy Grigoriev (GitHub: trianmon)
// SPDX-License-Identifier: Apache-2.0

#include <cstdint>

#include "ros2scope/plugin_api/core_context.hpp"
#include "ros2scope/plugin_api/iplugin.hpp"

namespace {

class ParamEditorPlugin final : public ros2scope::plugin_api::IPlugin {
public:
  const char *name() const override { return "param_editor"; }
  const char *version() const override { return "0.1.0"; }

  void onLoad(ros2scope::plugin_api::CoreContext *core_context) override { core_context_ = core_context; }
  void onUnload() override { core_context_ = nullptr; }

  void onUpdate(double dt) override { (void)dt; }
  void onDraw() override {}

private:
  ros2scope::plugin_api::CoreContext *core_context_{nullptr};
};

}  // namespace

extern "C" std::uint32_t ros2scope_plugin_api_version() {
  return ros2scope::plugin_api::kPluginApiVersion;
}

extern "C" ros2scope::plugin_api::IPlugin *create_plugin() { return new ParamEditorPlugin(); }
