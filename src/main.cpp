#include "ros2scope/app/application.hpp"

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>

namespace {

std::filesystem::path resolveExecutablePath(const char *argv0) {
  std::error_code ec;
  const auto proc_path = std::filesystem::read_symlink("/proc/self/exe", ec);
  if (!ec && !proc_path.empty()) {
    return std::filesystem::canonical(proc_path, ec);
  }

  std::filesystem::path input = argv0 == nullptr ? std::filesystem::path{} : std::filesystem::path(argv0);
  if (input.is_absolute()) {
    return input;
  }

  const char *path_env = std::getenv("PATH");
  if (path_env != nullptr) {
    std::stringstream ss(path_env);
    std::string entry;
    while (std::getline(ss, entry, ':')) {
      if (entry.empty()) {
        continue;
      }
      std::filesystem::path candidate = std::filesystem::path(entry) / input;
      if (std::filesystem::exists(candidate)) {
        return candidate;
      }
    }
  }

  return input;
}

void printHelp() {
  std::cout << "ROS 2 Scope\n"
            << "Usage: r2s [options]\n"
            << "  --layout <file>\n"
            << "  --plugin <name>\n"
            << "  --ros-domain-id <id>\n"
            << "  --no-plugins\n"
            << "  --safe-mode\n"
            << "  --help\n";
}

std::optional<ros2scope::app::AppOptions> parseArgs(int argc, char **argv) {
  ros2scope::app::AppOptions options;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];

    if (arg == "--layout" && i + 1 < argc) {
      options.layout_file = argv[++i];
      continue;
    }

    if (arg == "--plugin" && i + 1 < argc) {
      options.plugin_filters.emplace_back(argv[++i]);
      continue;
    }

    if (arg == "--ros-domain-id" && i + 1 < argc) {
      options.ros_domain_id = std::stoi(argv[++i]);
      continue;
    }

    if (arg == "--no-plugins") {
      options.no_plugins = true;
      continue;
    }

    if (arg == "--safe-mode") {
      options.safe_mode = true;
      options.no_plugins = true;
      continue;
    }

    if (arg == "--help") {
      printHelp();
      return std::nullopt;
    }

    std::cerr << "Unknown argument: " << arg << "\n";
    printHelp();
    return std::nullopt;
  }

  return options;
}

}  // namespace

int main(int argc, char **argv) {
  auto parsed = parseArgs(argc, argv);
  if (!parsed.has_value()) {
    return 0;
  }

  ros2scope::app::Application app(std::move(*parsed), resolveExecutablePath(argv[0]));
  return app.run(argc, argv);
}
