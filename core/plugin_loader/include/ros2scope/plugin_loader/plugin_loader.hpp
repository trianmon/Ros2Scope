// SPDX-FileCopyrightText: 2026 Georgy Grigoriev (GitHub: trianmon)
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "ros2scope/plugin_api/core_context.hpp"
#include "ros2scope/plugin_api/iplugin.hpp"

namespace ros2scope::plugin_loader {

struct PluginInstance {
	std::string id;
	std::filesystem::path path;
	void *handle{nullptr};
	std::unique_ptr<ros2scope::plugin_api::IPlugin> plugin;
};

struct PluginInfo {
	std::string id;
	std::filesystem::path path;
};

class PluginLoader {
public:
	PluginLoader(
			std::filesystem::path executable_path,
			std::vector<std::filesystem::path> extra_search_paths = {});

	std::vector<std::filesystem::path> discover() const;

	bool load(
			const std::filesystem::path &path,
			ros2scope::plugin_api::CoreContext *core_context,
			std::string *error = nullptr);

	bool unload(const std::string &plugin_id, std::string *error = nullptr);
	bool reload(const std::string &plugin_id, ros2scope::plugin_api::CoreContext *core_context, std::string *error = nullptr);

	void unloadAll();
	void updateAll(double dt);
	void drawAll();

	std::vector<std::string> loadedPluginIds() const;
	std::vector<PluginInfo> loadedPlugins() const;

private:
	std::vector<std::filesystem::path> searchPaths() const;

	std::filesystem::path executable_path_;
	std::vector<std::filesystem::path> extra_search_paths_;
	std::vector<PluginInstance> loaded_plugins_;
};

}  // namespace ros2scope::plugin_loader
