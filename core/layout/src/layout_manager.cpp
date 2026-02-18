// SPDX-FileCopyrightText: 2026 Georgy Grigoriev (GitHub: trianmon)
// SPDX-License-Identifier: Apache-2.0

#include "ros2scope/layout/layout_manager.hpp"

#include <filesystem>
#include <fstream>
#include <sstream>

namespace ros2scope::layout {

namespace {
std::string extractJsonString(const std::string &json, const std::string &key) {
	const std::string token = "\"" + key + "\"";
	const auto key_pos = json.find(token);
	if (key_pos == std::string::npos) {
		return {};
	}

	const auto colon_pos = json.find(':', key_pos + token.size());
	if (colon_pos == std::string::npos) {
		return {};
	}

	const auto begin_quote = json.find('"', colon_pos + 1);
	if (begin_quote == std::string::npos) {
		return {};
	}

	std::string out;
	out.reserve(json.size());

	bool escaped = false;
	for (std::size_t i = begin_quote + 1; i < json.size(); ++i) {
		const char ch = json[i];
		if (!escaped && ch == '"') {
			return out;
		}

		if (!escaped && ch == '\\') {
			escaped = true;
			continue;
		}

		if (escaped) {
			switch (ch) {
				case 'n':
					out.push_back('\n');
					break;
				case 'r':
					out.push_back('\r');
					break;
				case 't':
					out.push_back('\t');
					break;
				case '\\':
					out.push_back('\\');
					break;
				case '"':
					out.push_back('"');
					break;
				default:
					out.push_back(ch);
					break;
			}
			escaped = false;
			continue;
		}

		out.push_back(ch);
	}

	return {};
}

std::string escapeJson(const std::string &input) {
	std::string out;
	out.reserve(input.size() + 16);
	for (char ch : input) {
		switch (ch) {
			case '"':
				out += "\\\"";
				break;
			case '\\':
				out += "\\\\";
				break;
			case '\n':
				out += "\\n";
				break;
			case '\r':
				out += "\\r";
				break;
			case '\t':
				out += "\\t";
				break;
			default:
				out.push_back(ch);
				break;
		}
	}
	return out;
}
}  // namespace

LayoutManager::LayoutManager(std::filesystem::path storage_dir)
		: storage_dir_(std::move(storage_dir)) {
	std::filesystem::create_directories(storage_dir_);
}

bool LayoutManager::save(const LayoutState &state, const std::string &file_name) const {
	const auto path = storage_dir_ / file_name;
	std::ofstream file(path, std::ios::trunc);
	if (!file.good()) {
		return false;
	}

	file << "{\n"
			 << "  \"profile\": \"" << escapeJson(state.profile) << "\",\n"
			 << "  \"docking_data\": \"" << escapeJson(state.docking_data) << "\"\n"
			 << "}\n";
	return true;
}

std::optional<LayoutState> LayoutManager::load(const std::string &file_name) const {
	const auto path = storage_dir_ / file_name;
	std::ifstream file(path);
	if (!file.good()) {
		return std::nullopt;
	}

	std::stringstream buffer;
	buffer << file.rdbuf();
	LayoutState state;
	state.profile = extractJsonString(buffer.str(), "profile");
	state.docking_data = extractJsonString(buffer.str(), "docking_data");
	return state;
}

const std::filesystem::path &LayoutManager::storageDir() const { return storage_dir_; }

}  // namespace ros2scope::layout
