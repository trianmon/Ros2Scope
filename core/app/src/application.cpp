#include "ros2scope/app/application.hpp"

#include <chrono>
#include <cstdlib>
#include <string>
#include <unordered_map>

#include <GL/gl.h>
#include <GLFW/glfw3.h>

#include <imgui.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>

#include <rclcpp/executors/single_threaded_executor.hpp>

namespace ros2scope::app {

Application::Application(AppOptions options, std::filesystem::path executable_path)
		: options_(std::move(options)),
			executable_path_(std::move(executable_path)),
			layout_manager_(std::filesystem::temp_directory_path() / "ros2scope" / "layouts"),
			type_resolver_(workspace_env_),
			plugin_loader_(executable_path_),
			core_context_{&event_bus_, &layout_manager_, nullptr, &type_resolver_, &workspace_env_} {}

int Application::run(int argc, char **argv) {
	if (options_.ros_domain_id.has_value()) {
		const std::string value = std::to_string(*options_.ros_domain_id);
		setenv("ROS_DOMAIN_ID", value.c_str(), 1);
	}

	rclcpp::init(argc, argv);
	node_ = std::make_shared<rclcpp::Node>("ros2scope");
	ros_bridge_ = std::make_unique<ros2scope::ros_bridge::RosBridge>(node_);
	core_context_.ros_bridge = ros_bridge_.get();

	rclcpp::executors::SingleThreadedExecutor executor;
	executor.add_node(node_);

	if (!glfwInit()) {
		rclcpp::shutdown();
		return 1;
	}

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	GLFWwindow *window = glfwCreateWindow(1600, 900, "ROS 2 Scope", nullptr, nullptr);
	if (window == nullptr) {
		glfwTerminate();
		rclcpp::shutdown();
		return 1;
	}

	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO &io = ImGui::GetIO();
	io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
	ImGui::StyleColorsDark();

	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init("#version 330");

	auto active_layout = layout_manager_.load(options_.layout_file).value_or(
			ros2scope::layout::LayoutState{.profile = "default", .docking_data = {}});
	if (!active_layout.docking_data.empty()) {
		ImGui::LoadIniSettingsFromMemory(active_layout.docking_data.c_str(), active_layout.docking_data.size());
	}

	if (!options_.safe_mode && !options_.no_plugins) {
		for (const auto &path : plugin_loader_.discover()) {
			if (!shouldLoadPlugin(path)) {
				continue;
			}

			std::string error;
			if (!plugin_loader_.load(path, &core_context_, &error)) {
				plugin_status_[path.filename().string()] = "load failed: " + error;
			}
		}
	}

	auto last_tick = std::chrono::steady_clock::now();
	while (rclcpp::ok() && !glfwWindowShouldClose(window)) {
		const auto now = std::chrono::steady_clock::now();
		const std::chrono::duration<double> delta = now - last_tick;
		last_tick = now;

		glfwPollEvents();
		executor.spin_some(std::chrono::milliseconds(1));

		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		const ImGuiViewport *viewport = ImGui::GetMainViewport();
		ImGui::SetNextWindowPos(viewport->WorkPos);
		ImGui::SetNextWindowSize(viewport->WorkSize);
		ImGui::SetNextWindowViewport(viewport->ID);
		ImGuiWindowFlags host_flags =
				ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoTitleBar |
				ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
				ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;

		ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
		ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
		ImGui::Begin("MainDockHost", nullptr, host_flags);
		ImGui::PopStyleVar(2);

		if (ImGui::BeginMenuBar()) {
			if (ImGui::BeginMenu("Layout")) {
				if (ImGui::MenuItem("Save now")) {
					active_layout.docking_data = ImGui::SaveIniSettingsToMemory();
					active_layout.profile = options_.layout_file;
					layout_manager_.save(active_layout, options_.layout_file);
				}
				ImGui::EndMenu();
			}
			ImGui::EndMenuBar();
		}

		ImGuiID dockspace_id = ImGui::GetID("R2S_DockSpace");
		ImGui::DockSpace(dockspace_id, ImVec2(0.0f, 0.0f), ImGuiDockNodeFlags_PassthruCentralNode);
		ImGui::End();

		ImGui::Begin("Plugin Manager");
		ImGui::Text("Discovered plugins:");

		const auto loaded_plugins = plugin_loader_.loadedPlugins();
		std::unordered_map<std::string, std::string> loaded_file_to_id;
		loaded_file_to_id.reserve(loaded_plugins.size());
		for (const auto &plugin : loaded_plugins) {
			loaded_file_to_id[plugin.path.stem().string()] = plugin.id;
		}

		for (const auto &path : plugin_loader_.discover()) {
			const std::string display_name = path.stem().string();
			const auto id_it = loaded_file_to_id.find(display_name);
			const bool is_loaded = id_it != loaded_file_to_id.end();
			const std::string plugin_id = is_loaded ? id_it->second : display_name;

			ImGui::PushID(display_name.c_str());
			ImGui::TextUnformatted(display_name.c_str());
			ImGui::SameLine(280.0f);

			if (!is_loaded) {
				if (ImGui::Button("Load")) {
					std::string error;
					if (!plugin_loader_.load(path, &core_context_, &error)) {
						plugin_status_[display_name] = "load failed: " + error;
					} else {
						plugin_status_.erase(display_name);
					}
				}
			} else {
				if (ImGui::Button("Reload")) {
					std::string error;
					if (!plugin_loader_.reload(plugin_id, &core_context_, &error)) {
						plugin_status_[display_name] = "reload failed: " + error;
					} else {
						plugin_status_.erase(display_name);
					}
				}
				ImGui::SameLine();
				if (ImGui::Button("Unload")) {
					std::string error;
					if (!plugin_loader_.unload(plugin_id, &error)) {
						plugin_status_[display_name] = "unload failed: " + error;
					} else {
						plugin_status_.erase(display_name);
					}
				}
			}

			const auto status_it = plugin_status_.find(display_name);
			if (status_it != plugin_status_.end()) {
				ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "%s", status_it->second.c_str());
			}
			ImGui::PopID();
		}

		ImGui::Separator();
		ImGui::Text("Loaded plugins: %zu", loaded_plugins.size());
		ImGui::Text("Frame dt: %.2f ms", delta.count() * 1000.0);
		ImGui::End();

		plugin_loader_.updateAll(delta.count());
		plugin_loader_.drawAll();

		ImGui::Render();
		int display_width = 0;
		int display_height = 0;
		glfwGetFramebufferSize(window, &display_width, &display_height);
		glViewport(0, 0, display_width, display_height);
		glClearColor(0.09f, 0.09f, 0.11f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
		glfwSwapBuffers(window);
	}

	active_layout.docking_data = ImGui::SaveIniSettingsToMemory();
	active_layout.profile = options_.layout_file;

	plugin_loader_.unloadAll();
	layout_manager_.save(active_layout, options_.layout_file);

	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();

	glfwDestroyWindow(window);
	glfwTerminate();

	rclcpp::shutdown();
	ros_bridge_.reset();
	node_.reset();
	return 0;
}

bool Application::shouldLoadPlugin(const std::filesystem::path &plugin_path) const {
	if (options_.plugin_filters.empty()) {
		return true;
	}

	const auto stem = plugin_path.stem().string();
	for (const auto &filter : options_.plugin_filters) {
		if (stem.find(filter) != std::string::npos) {
			return true;
		}
	}

	return false;
}

}  // namespace ros2scope::app
