// SPDX-FileCopyrightText: 2026 Georgy Grigoriev (GitHub: trianmon)
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <filesystem>
#include <optional>
#include <string>

namespace ros2scope::layout {

struct LayoutState {
  std::string profile{"default"};
  std::string docking_data;
};

class LayoutManager {
public:
  explicit LayoutManager(std::filesystem::path storage_dir);

  bool save(const LayoutState &state, const std::string &file_name) const;
  std::optional<LayoutState> load(const std::string &file_name) const;

  const std::filesystem::path &storageDir() const;

private:
  std::filesystem::path storage_dir_;
};

}  // namespace ros2scope::layout
