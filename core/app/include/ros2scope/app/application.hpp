// SPDX-FileCopyrightText: 2026 Georgy Grigoriev (GitHub: trianmon)
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "ros2scope/event_bus/event_bus.hpp"
#include "ros2scope/layout/layout_manager.hpp"
#include "ros2scope/plugin_api/core_context.hpp"
#include "ros2scope/plugin_loader/plugin_loader.hpp"
#include "ros2scope/ros_bridge/ros_bridge.hpp"
#include "ros2scope/type_introspection/type_resolver.hpp"
#include "ros2scope/workspace_env/workspace_environment.hpp"

namespace ros2scope::app {

struct AppOptions {
	std::string layout_file{"default.json"};
	std::vector<std::string> plugin_filters;
	std::optional<int> ros_domain_id;
	bool no_plugins{false};
	bool safe_mode{false};
};

class Application {
public:
	Application(AppOptions options, std::filesystem::path executable_path);

	int run(int argc, char **argv);

private:
	bool shouldLoadPlugin(const std::filesystem::path &plugin_path) const;

	AppOptions options_;
	std::filesystem::path executable_path_;

	std::shared_ptr<rclcpp::Node> node_;
	ros2scope::event_bus::EventBus event_bus_;
	ros2scope::workspace_env::WorkspaceEnvironment workspace_env_;
	ros2scope::layout::LayoutManager layout_manager_;
	std::unique_ptr<ros2scope::ros_bridge::RosBridge> ros_bridge_;
	ros2scope::type_introspection::TypeResolver type_resolver_;
	ros2scope::plugin_loader::PluginLoader plugin_loader_;
	ros2scope::plugin_api::CoreContext core_context_;
	std::unordered_map<std::string, std::string> plugin_status_;
};

}  // namespace ros2scope::app
