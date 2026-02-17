#include "ros2scope/plugin_loader/plugin_loader.hpp"

#include <algorithm>
#include <cstdlib>
#include <sstream>
#include <unordered_set>

#include <dlfcn.h>

namespace ros2scope::plugin_loader {

namespace {
std::vector<std::filesystem::path> splitPathList(const char *value) {
	std::vector<std::filesystem::path> output;
	if (value == nullptr || value[0] == '\0') {
		return output;
	}

	std::stringstream stream(value);
	std::string token;
	while (std::getline(stream, token, ':')) {
		if (!token.empty()) {
			output.emplace_back(token);
		}
	}
	return output;
}
}  // namespace

PluginLoader::PluginLoader(
		std::filesystem::path executable_path,
		std::vector<std::filesystem::path> extra_search_paths)
		: executable_path_(std::move(executable_path)), extra_search_paths_(std::move(extra_search_paths)) {}

std::vector<std::filesystem::path> PluginLoader::searchPaths() const {
	std::vector<std::filesystem::path> paths;

	if (!executable_path_.empty()) {
		const auto prefix = executable_path_.parent_path().parent_path();
		paths.emplace_back(prefix / "lib" / "ros2scope" / "plugins");
	}

	const auto env_paths = splitPathList(std::getenv("R2S_PLUGIN_PATH"));
	paths.insert(paths.end(), env_paths.begin(), env_paths.end());

	if (const char *home = std::getenv("HOME"); home != nullptr && home[0] != '\0') {
		paths.emplace_back(std::filesystem::path(home) / ".local" / "lib" / "ros2scope" / "plugins");
	}
	paths.emplace_back("/usr/local/lib/ros2scope/plugins");
	paths.emplace_back("/usr/lib/ros2scope/plugins");

	paths.insert(paths.end(), extra_search_paths_.begin(), extra_search_paths_.end());

	return paths;
}

std::vector<std::filesystem::path> PluginLoader::discover() const {
	std::vector<std::filesystem::path> plugins;
	std::unordered_set<std::string> seen_paths;
	std::unordered_set<std::string> seen_plugin_filenames;

	for (const auto &raw_path : searchPaths()) {
		std::error_code ec;
		const auto path = std::filesystem::weakly_canonical(raw_path, ec);
		const auto normalized_path = ec ? raw_path.lexically_normal() : path;
		const auto normalized_path_key = normalized_path.string();

		if (!seen_paths.insert(normalized_path_key).second) {
			continue;
		}

		if (!std::filesystem::exists(path) || !std::filesystem::is_directory(path)) {
			continue;
		}

		for (const auto &entry : std::filesystem::directory_iterator(path)) {
			if (!entry.is_regular_file()) {
				continue;
			}

			const auto &plugin_path = entry.path();
			if (plugin_path.extension() == ".so") {
				const auto plugin_filename = plugin_path.filename().string();
				if (!seen_plugin_filenames.insert(plugin_filename).second) {
					continue;
				}
				plugins.push_back(plugin_path);
			}
		}
	}

	std::sort(plugins.begin(), plugins.end());
	return plugins;
}

bool PluginLoader::load(
		const std::filesystem::path &path,
		ros2scope::plugin_api::CoreContext *core_context,
		std::string *error) {
	void *handle = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
	if (handle == nullptr) {
		if (error != nullptr) {
			*error = dlerror();
		}
		return false;
	}

	auto get_api_version = reinterpret_cast<ros2scope::plugin_api::GetApiVersionFn>(
			dlsym(handle, "ros2scope_plugin_api_version"));
	if (get_api_version == nullptr || get_api_version() != ros2scope::plugin_api::kPluginApiVersion) {
		if (error != nullptr) {
			*error = "Plugin API version mismatch";
		}
		dlclose(handle);
		return false;
	}

	auto create_plugin = reinterpret_cast<ros2scope::plugin_api::CreatePluginFn>(
			dlsym(handle, "create_plugin"));
	if (create_plugin == nullptr) {
		if (error != nullptr) {
			*error = "Symbol create_plugin not found";
		}
		dlclose(handle);
		return false;
	}

	auto raw_plugin = create_plugin();
	if (raw_plugin == nullptr) {
		if (error != nullptr) {
			*error = "Plugin returned nullptr";
		}
		dlclose(handle);
		return false;
	}

	std::unique_ptr<ros2scope::plugin_api::IPlugin> plugin(raw_plugin);
	plugin->onLoad(core_context);

	PluginInstance instance;
	instance.id = plugin->name();
	instance.path = path;
	instance.handle = handle;
	instance.plugin = std::move(plugin);
	loaded_plugins_.push_back(std::move(instance));
	return true;
}

bool PluginLoader::unload(const std::string &plugin_id, std::string *error) {
	const auto it = std::find_if(loaded_plugins_.begin(), loaded_plugins_.end(), [&](const PluginInstance &p) {
		return p.id == plugin_id;
	});

	if (it == loaded_plugins_.end()) {
		if (error != nullptr) {
			*error = "Plugin not loaded";
		}
		return false;
	}

	it->plugin->onUnload();
	it->plugin.reset();
	if (dlclose(it->handle) != 0 && error != nullptr) {
		*error = dlerror();
	}

	loaded_plugins_.erase(it);
	return true;
}

bool PluginLoader::reload(
		const std::string &plugin_id,
		ros2scope::plugin_api::CoreContext *core_context,
		std::string *error) {
	const auto it = std::find_if(loaded_plugins_.begin(), loaded_plugins_.end(), [&](const PluginInstance &p) {
		return p.id == plugin_id;
	});

	if (it == loaded_plugins_.end()) {
		if (error != nullptr) {
			*error = "Plugin not loaded";
		}
		return false;
	}

	const auto path = it->path;
	if (!unload(plugin_id, error)) {
		return false;
	}

	return load(path, core_context, error);
}

void PluginLoader::unloadAll() {
	while (!loaded_plugins_.empty()) {
		auto plugin_id = loaded_plugins_.back().id;
		unload(plugin_id, nullptr);
	}
}

void PluginLoader::updateAll(double dt) {
	for (auto &instance : loaded_plugins_) {
		instance.plugin->onUpdate(dt);
	}
}

void PluginLoader::drawAll() {
	for (auto &instance : loaded_plugins_) {
		instance.plugin->onDraw();
	}
}

std::vector<std::string> PluginLoader::loadedPluginIds() const {
	std::vector<std::string> ids;
	ids.reserve(loaded_plugins_.size());
	for (const auto &plugin : loaded_plugins_) {
		ids.push_back(plugin.id);
	}
	return ids;
}

std::vector<PluginInfo> PluginLoader::loadedPlugins() const {
	std::vector<PluginInfo> plugins;
	plugins.reserve(loaded_plugins_.size());
	for (const auto &plugin : loaded_plugins_) {
		plugins.push_back(PluginInfo{.id = plugin.id, .path = plugin.path});
	}
	return plugins;
}

}  // namespace ros2scope::plugin_loader
