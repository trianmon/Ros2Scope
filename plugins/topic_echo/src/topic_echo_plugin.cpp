// SPDX-FileCopyrightText: 2026 Georgy Grigoriev (GitHub: trianmon)
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <imgui.h>

#include <rclcpp/serialization.hpp>
#include <rclcpp/typesupport_helpers.hpp>

#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

#include "ros2scope/plugin_api/core_context.hpp"
#include "ros2scope/plugin_api/iplugin.hpp"
#include "ros2scope/ros_bridge/ros_bridge.hpp"
#include "ros2scope/type_introspection/type_resolver.hpp"

namespace {

struct FieldTreeNode {
  std::string type;
  std::string name;
  std::vector<FieldTreeNode> children;
};

struct ValueTreeNode {
  std::string name;
  std::string value;
  std::vector<ValueTreeNode> children;
};

std::string trim(const std::string &value) {
  const auto begin = value.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return {};
  }
  const auto end = value.find_last_not_of(" \t\r\n");
  return value.substr(begin, end - begin + 1);
}

void drawFieldTreeNode(const FieldTreeNode &node) {
  const std::string label = node.name.empty() ? node.type : (node.name + " : " + node.type);
  if (node.children.empty()) {
    ImGui::BulletText("%s", label.c_str());
    return;
  }

  if (ImGui::TreeNode(label.c_str())) {
    for (const auto &child : node.children) {
      drawFieldTreeNode(child);
    }
    ImGui::TreePop();
  }
}

void drawValueTreeNode(const ValueTreeNode &node) {
  const std::string label = node.value.empty() ? node.name : (node.name + ": " + node.value);
  if (node.children.empty()) {
    ImGui::BulletText("%s", label.c_str());
    return;
  }

  if (ImGui::TreeNode(label.c_str())) {
    for (const auto &child : node.children) {
      drawValueTreeNode(child);
    }
    ImGui::TreePop();
  }
}

std::string scalarToString(uint8_t type_id, const void *data) {
  std::ostringstream stream;

  switch (type_id) {
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT:
      stream << *static_cast<const float *>(data);
      return stream.str();
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE:
      stream << *static_cast<const double *>(data);
      return stream.str();
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_LONG_DOUBLE:
      stream << static_cast<double>(*static_cast<const long double *>(data));
      return stream.str();
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR:
      return std::to_string(static_cast<int>(*static_cast<const unsigned char *>(data)));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_OCTET:
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
      return std::to_string(static_cast<unsigned>(*static_cast<const uint8_t *>(data)));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
      return std::to_string(static_cast<int>(*static_cast<const int8_t *>(data)));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
      return std::to_string(*static_cast<const uint16_t *>(data));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
      return std::to_string(*static_cast<const int16_t *>(data));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
      return std::to_string(*static_cast<const uint32_t *>(data));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
      return std::to_string(*static_cast<const int32_t *>(data));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
      return std::to_string(*static_cast<const uint64_t *>(data));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
      return std::to_string(*static_cast<const int64_t *>(data));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN:
      return *static_cast<const bool *>(data) ? "true" : "false";
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING:
      return *static_cast<const std::string *>(data);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING: {
      const auto &wstr = *static_cast<const std::u16string *>(data);
      return "u16string(len=" + std::to_string(wstr.size()) + ")";
    }
    default:
      return "<unsupported>";
  }
}

std::string fieldTypeToString(const rosidl_typesupport_introspection_cpp::MessageMember &member) {
  std::string type_name;
  switch (member.type_id_) {
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT:
      type_name = "float32";
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE:
      type_name = "float64";
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_LONG_DOUBLE:
      type_name = "long double";
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR:
      type_name = "char";
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_OCTET:
      type_name = "octet";
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
      type_name = "uint8";
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
      type_name = "int8";
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
      type_name = "uint16";
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
      type_name = "int16";
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
      type_name = "uint32";
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
      type_name = "int32";
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
      type_name = "uint64";
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
      type_name = "int64";
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN:
      type_name = "bool";
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING:
      type_name = "string";
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING:
      type_name = "wstring";
      break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE: {
      const auto *sub_members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
          member.members_ != nullptr ? member.members_->data : nullptr);
      if (sub_members != nullptr) {
        type_name = std::string(sub_members->message_namespace_) + "/" + sub_members->message_name_;
      } else {
        type_name = "message";
      }
      break;
    }
    default:
      type_name = "unknown";
      break;
  }

  if (member.is_array_) {
    if (!member.is_upper_bound_ && member.array_size_ > 0) {
      type_name += "[" + std::to_string(member.array_size_) + "]";
    } else if (member.is_upper_bound_ && member.array_size_ > 0) {
      type_name += "[<=" + std::to_string(member.array_size_) + "]";
    } else {
      type_name += "[]";
    }
  }

  return type_name;
}

class TopicEchoPlugin final : public ros2scope::plugin_api::IPlugin {
public:
  const char *name() const override { return "topic_echo"; }
  const char *version() const override { return "0.2.0"; }

  void onLoad(ros2scope::plugin_api::CoreContext *core_context) override {
    core_context_ = core_context;
    last_topic_refresh_ = std::chrono::steady_clock::now() - std::chrono::seconds(10);
  }

  void onUnload() override {
    unsubscribeCurrentTopic();
    clearMessages();
    prepared_type_.clear();
    serializer_.reset();
    cpp_typesupport_ = nullptr;
    introspection_typesupport_ = nullptr;
    cpp_typesupport_library_.reset();
    introspection_typesupport_library_.reset();
    core_context_ = nullptr;
  }

  void onUpdate(double dt) override {
    (void)dt;

    if (core_context_ == nullptr || core_context_->ros_bridge == nullptr) {
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (now - last_topic_refresh_ >= std::chrono::milliseconds(500)) {
      topics_ = core_context_->ros_bridge->listTopics();
      last_topic_refresh_ = now;

      if (!selected_topic_.empty()) {
        bool still_exists = false;
        for (const auto &topic : topics_) {
          if (topic.name == selected_topic_ && topic.type == selected_type_) {
            still_exists = true;
            break;
          }
        }
        if (!still_exists) {
          unsubscribeCurrentTopic();
          selected_topic_.clear();
          selected_type_.clear();
          value_tree_.clear();
          value_error_.clear();
        }
      }
    }

  }

  void onDraw() override {
    if (!ImGui::Begin("Topic Echo")) {
      ImGui::End();
      return;
    }

    if (core_context_ == nullptr || core_context_->ros_bridge == nullptr) {
      ImGui::TextUnformatted("ROS bridge is not available.");
      ImGui::End();
      return;
    }

    ImGui::Text("Topics: %zu", topics_.size());
    ImGui::SameLine();
    if (ImGui::Button("Refresh")) {
      topics_ = core_context_->ros_bridge->listTopics();
      last_topic_refresh_ = std::chrono::steady_clock::now();
    }

    ImGui::Separator();
    ImGui::TextUnformatted("Topic List");
    ImGui::BeginChild("topic_list", ImVec2(0.0f, 180.0f), true);
    for (const auto &topic : topics_) {
      const bool selected = (topic.name == selected_topic_ && topic.type == selected_type_);

      std::ostringstream label;
      label << topic.name << "  [" << topic.type << "]  pub=" << topic.publishers << " sub=" << topic.subscribers;
      if (ImGui::Selectable(label.str().c_str(), selected)) {
        if (selected_topic_ != topic.name || selected_type_ != topic.type) {
          unsubscribeCurrentTopic();
          selected_topic_ = topic.name;
          selected_type_ = topic.type;
          clearMessages();
          value_tree_.clear();
          value_error_.clear();
        }
      }
    }
    ImGui::EndChild();

    ImGui::Separator();
    ImGui::Text("Selected: %s", selected_topic_.empty() ? "<none>" : selected_topic_.c_str());
    ImGui::Text("Type: %s", selected_type_.empty() ? "<none>" : selected_type_.c_str());

    if (!selected_type_.empty() && selected_type_ != last_schema_type_) {
      refreshFieldTree();
    }

    ImGui::SameLine();
    if (ImGui::Button("Refresh fields") && !selected_type_.empty()) {
      refreshFieldTree();
    }

    ImGui::SliderInt("Throttle (ms)", &throttle_ms_, 0, 1000);
    ImGui::SliderInt("Decode throttle (ms)", &decode_throttle_ms_, 0, 2000);
    ImGui::SliderInt("Max array items", &max_array_items_, 0, 512);
    ImGui::Checkbox("Autoscroll", &autoscroll_);
    ImGui::Checkbox("Pause", &paused_);
    ImGui::SliderInt("Max messages", &max_messages_, 50, 5000);

    if (!selected_topic_.empty() && !subscribed_) {
      if (ImGui::Button("Subscribe")) {
        subscribeCurrentTopic();
      }
    } else if (subscribed_) {
      if (ImGui::Button("Unsubscribe")) {
        unsubscribeCurrentTopic();
      }
    }
    ImGui::SameLine();
    if (ImGui::Button("Clear")) {
      clearMessages();
    }

    ImGui::Separator();
    ImGui::TextUnformatted("Field Tree");
    ImGui::BeginChild("field_tree", ImVec2(0.0f, 180.0f), true);
    if (selected_type_.empty()) {
      ImGui::TextUnformatted("Select topic to inspect fields.");
    } else if (!schema_error_.empty()) {
      ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "%s", schema_error_.c_str());
    } else if (field_tree_.empty()) {
      ImGui::TextUnformatted("No fields parsed.");
    } else {
      for (const auto &root : field_tree_) {
        drawFieldTreeNode(root);
      }
    }
    ImGui::EndChild();

    ImGui::Separator();
    ImGui::TextUnformatted("Value Tree");
    ImGui::BeginChild("value_tree", ImVec2(0.0f, 180.0f), true);
    std::string local_value_error;
    std::vector<ValueTreeNode> local_value_tree;
    {
      std::scoped_lock lock(value_mutex_);
      local_value_error = value_error_;
      local_value_tree = value_tree_;
    }

    if (selected_topic_.empty()) {
      ImGui::TextUnformatted("Select topic to inspect values.");
    } else if (!local_value_error.empty()) {
      ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "%s", local_value_error.c_str());
    } else {
      if (local_value_tree.empty()) {
        ImGui::TextUnformatted("No decoded values yet. Subscribe and wait for messages.");
      }
      for (const auto &root : local_value_tree) {
        drawValueTreeNode(root);
      }
    }
    ImGui::EndChild();

    ImGui::Separator();
    ImGui::Text("Received: %llu", static_cast<unsigned long long>(message_counter_.load()));
    ImGui::SameLine();
    ImGui::Text("Decoded: %llu", static_cast<unsigned long long>(decoded_counter_.load()));

    ImGui::BeginChild("message_view", ImVec2(0.0f, 0.0f), true);
    {
      std::scoped_lock lock(messages_mutex_);
      for (const auto &line : messages_) {
        ImGui::TextWrapped("%s", line.c_str());
      }

      if (autoscroll_ && ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 20.0f) {
        ImGui::SetScrollHereY(1.0f);
      }
    }
    ImGui::EndChild();

    ImGui::End();
  }

  bool prepareTypeSupportForSelectedType() {
    if (selected_type_.empty()) {
      return false;
    }

    if (prepared_type_ == selected_type_ && serializer_ != nullptr && introspection_typesupport_ != nullptr) {
      return true;
    }

    serializer_.reset();
    cpp_typesupport_ = nullptr;
    introspection_typesupport_ = nullptr;
    cpp_typesupport_library_.reset();
    introspection_typesupport_library_.reset();
    prepared_type_.clear();

    try {
      cpp_typesupport_library_ = rclcpp::get_typesupport_library(selected_type_, "rosidl_typesupport_cpp");
      cpp_typesupport_ = rclcpp::get_message_typesupport_handle(
          selected_type_, "rosidl_typesupport_cpp", *cpp_typesupport_library_);
      serializer_ = std::make_unique<rclcpp::SerializationBase>(cpp_typesupport_);

      introspection_typesupport_library_ =
          rclcpp::get_typesupport_library(selected_type_, "rosidl_typesupport_introspection_cpp");
      introspection_typesupport_ = rclcpp::get_message_typesupport_handle(
          selected_type_, "rosidl_typesupport_introspection_cpp", *introspection_typesupport_library_);

      prepared_type_ = selected_type_;
      {
        std::scoped_lock lock(value_mutex_);
        value_error_.clear();
      }
      return true;
    } catch (const std::exception &e) {
      std::scoped_lock lock(value_mutex_);
      value_error_ = std::string("Type support error: ") + e.what();
      return false;
    }
  }

  ValueTreeNode buildValueNodeFromMember(
      const rosidl_typesupport_introspection_cpp::MessageMember &member,
      const void *field_ptr,
      const std::string &label) const {
    ValueTreeNode node;
    node.name = label;

    if (member.is_array_) {
      std::size_t item_count = member.array_size_;
      if (member.size_function != nullptr) {
        item_count = member.size_function(field_ptr);
      }

      node.value = "size=" + std::to_string(item_count);
      const std::size_t array_limit = static_cast<std::size_t>(std::max(0, max_array_items_));
      if (item_count > array_limit) {
        node.value += ", skipped large array (limit=" + std::to_string(array_limit) + ")";
        return node;
      }

      for (std::size_t index = 0; index < item_count; ++index) {
        const void *item_ptr = member.get_const_function != nullptr ? member.get_const_function(field_ptr, index) : nullptr;
        if (item_ptr == nullptr) {
          continue;
        }

        const std::string item_label = "[" + std::to_string(index) + "]";
        if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
          const auto *sub_members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
              member.members_ != nullptr ? member.members_->data : nullptr);
          if (sub_members == nullptr) {
            node.children.push_back(ValueTreeNode{.name = item_label, .value = "<missing submessage members>"});
            continue;
          }

          ValueTreeNode child;
          child.name = item_label;
          for (uint32_t i = 0; i < sub_members->member_count_; ++i) {
            const auto &sub = sub_members->members_[i];
            const auto *sub_ptr = static_cast<const uint8_t *>(item_ptr) + sub.offset_;
            child.children.push_back(buildValueNodeFromMember(sub, sub_ptr, sub.name_));
          }
          node.children.push_back(std::move(child));
        } else {
          node.children.push_back(ValueTreeNode{.name = item_label, .value = scalarToString(member.type_id_, item_ptr)});
        }
      }
      return node;
    }

    if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
      const auto *sub_members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
          member.members_ != nullptr ? member.members_->data : nullptr);
      if (sub_members == nullptr) {
        node.value = "<missing submessage members>";
        return node;
      }

      for (uint32_t i = 0; i < sub_members->member_count_; ++i) {
        const auto &sub = sub_members->members_[i];
        const auto *sub_ptr = static_cast<const uint8_t *>(field_ptr) + sub.offset_;
        node.children.push_back(buildValueNodeFromMember(sub, sub_ptr, sub.name_));
      }
      return node;
    }

    node.value = scalarToString(member.type_id_, field_ptr);
    return node;
  }

  FieldTreeNode buildFieldNodeFromMember(
      const rosidl_typesupport_introspection_cpp::MessageMember &member,
      const std::string &label) const {
    FieldTreeNode node;
    node.name = label;
    node.type = fieldTypeToString(member);

    if (member.type_id_ != rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
      return node;
    }

    const auto *sub_members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
        member.members_ != nullptr ? member.members_->data : nullptr);
    if (sub_members == nullptr) {
      return node;
    }

    if (member.is_array_) {
      FieldTreeNode item;
      item.name = "item";
      item.type = std::string(sub_members->message_namespace_) + "/" + sub_members->message_name_;
      for (uint32_t i = 0; i < sub_members->member_count_; ++i) {
        const auto &sub = sub_members->members_[i];
        item.children.push_back(buildFieldNodeFromMember(sub, sub.name_));
      }
      node.children.push_back(std::move(item));
      return node;
    }

    for (uint32_t i = 0; i < sub_members->member_count_; ++i) {
      const auto &sub = sub_members->members_[i];
      node.children.push_back(buildFieldNodeFromMember(sub, sub.name_));
    }

    return node;
  }

  void updateValueTreeFromSerialized(const rclcpp::SerializedMessage &message) {
    if (!prepareTypeSupportForSelectedType()) {
      return;
    }

    const auto *message_members =
        static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(introspection_typesupport_->data);
    if (message_members == nullptr) {
      std::scoped_lock lock(value_mutex_);
      value_error_ = "Missing introspection message members";
      return;
    }

    std::unique_ptr<unsigned char[]> message_data(new unsigned char[message_members->size_of_]);
    std::memset(message_data.get(), 0, message_members->size_of_);
    message_members->init_function(message_data.get(), rosidl_runtime_cpp::MessageInitialization::ALL);

    try {
      serializer_->deserialize_message(&message, message_data.get());
    } catch (const std::exception &e) {
      message_members->fini_function(message_data.get());
      std::scoped_lock lock(value_mutex_);
      value_error_ = std::string("Deserialize error: ") + e.what();
      return;
    }

    std::vector<ValueTreeNode> new_tree;

    ValueTreeNode meta;
    meta.name = "meta";
    meta.children.push_back(ValueTreeNode{.name = "topic", .value = selected_topic_});
    meta.children.push_back(ValueTreeNode{.name = "type", .value = selected_type_});
    meta.children.push_back(ValueTreeNode{
        .name = "size_bytes",
        .value = std::to_string(message.get_rcl_serialized_message().buffer_length)});
    new_tree.push_back(std::move(meta));

    ValueTreeNode root;
    root.name = "message";
    for (uint32_t i = 0; i < message_members->member_count_; ++i) {
      const auto &member = message_members->members_[i];
      const auto *field_ptr = message_data.get() + member.offset_;
      root.children.push_back(buildValueNodeFromMember(member, field_ptr, member.name_));
    }
    new_tree.push_back(std::move(root));

    {
      std::scoped_lock lock(value_mutex_);
      value_tree_ = std::move(new_tree);
      value_error_.clear();
    }
    message_members->fini_function(message_data.get());
  }

  void refreshFieldTree() {
    schema_error_.clear();
    field_tree_.clear();
    last_schema_type_ = selected_type_;

    if (selected_type_.empty()) {
      return;
    }

    if (!prepareTypeSupportForSelectedType()) {
      std::string local_value_error;
      {
        std::scoped_lock lock(value_mutex_);
        local_value_error = value_error_;
      }
      schema_error_ = local_value_error.empty() ? "Failed to prepare type support" : local_value_error;
      return;
    }

    const auto *message_members =
        static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(introspection_typesupport_->data);
    if (message_members == nullptr) {
      schema_error_ = "Missing introspection message members";
      return;
    }

    for (uint32_t i = 0; i < message_members->member_count_; ++i) {
      const auto &member = message_members->members_[i];
      field_tree_.push_back(buildFieldNodeFromMember(member, member.name_));
    }
  }

private:
  void clearMessages() {
    std::scoped_lock lock(messages_mutex_);
    messages_.clear();
    message_counter_.store(0);
    decoded_counter_.store(0);
  }

  std::string formatSerialized(const rclcpp::SerializedMessage &message) const {
    const auto &raw = message.get_rcl_serialized_message();
    const auto *buffer = raw.buffer;
    const std::size_t size = raw.buffer_length;

    constexpr std::size_t kPreviewBytes = 48;
    const std::size_t count = std::min(size, kPreviewBytes);

    std::ostringstream stream;
    stream << "size=" << size << " bytes | ";
    stream << std::hex << std::setfill('0');
    for (std::size_t i = 0; i < count; ++i) {
      stream << std::setw(2) << static_cast<unsigned>(buffer[i]);
      if (i + 1 < count) {
        stream << ' ';
      }
    }
    if (size > kPreviewBytes) {
      stream << " ...";
    }
    return stream.str();
  }

  void subscribeCurrentTopic() {
    if (selected_topic_.empty() || selected_type_.empty()) {
      return;
    }

    if (core_context_ == nullptr || core_context_->ros_bridge == nullptr) {
      return;
    }

    if (core_context_->type_resolver != nullptr) {
      core_context_->type_resolver->ensureTypeSupportLoaded(selected_type_);
    }

    prepareTypeSupportForSelectedType();

    last_emit_time_ = std::chrono::steady_clock::time_point{};
    last_decode_time_ = std::chrono::steady_clock::time_point{};
    const bool ok = core_context_->ros_bridge->subscribeGeneric(
        selected_topic_,
        selected_type_,
        [this](const rclcpp::SerializedMessage &message) {
          if (paused_) {
            return;
          }

          const int throttle_value = throttle_ms_;
          const auto now = std::chrono::steady_clock::now();
          if (throttle_value > 0 && last_emit_time_ != std::chrono::steady_clock::time_point{} &&
              now - last_emit_time_ < std::chrono::milliseconds(throttle_value)) {
            return;
          }
          last_emit_time_ = now;

          std::ostringstream prefix;
          prefix << "#" << (message_counter_.load() + 1) << " [" << selected_topic_ << "] ";
          auto line = prefix.str() + formatSerialized(message);

          {
            std::scoped_lock lock(messages_mutex_);
            messages_.push_back(std::move(line));
            while (messages_.size() > static_cast<std::size_t>(std::max(10, max_messages_))) {
              messages_.pop_front();
            }
          }
          message_counter_.fetch_add(1);

          const int decode_throttle = decode_throttle_ms_;
          const auto decode_now = std::chrono::steady_clock::now();
          if (decode_throttle > 0 && last_decode_time_ != std::chrono::steady_clock::time_point{} &&
              decode_now - last_decode_time_ < std::chrono::milliseconds(decode_throttle)) {
            return;
          }

          last_decode_time_ = decode_now;
          updateValueTreeFromSerialized(message);
          decoded_counter_.fetch_add(1);
        });

    subscribed_ = ok;
  }

  void unsubscribeCurrentTopic() {
    if (!subscribed_ || core_context_ == nullptr || core_context_->ros_bridge == nullptr || selected_topic_.empty()) {
      subscribed_ = false;
      return;
    }

    core_context_->ros_bridge->unsubscribe(selected_topic_);
    subscribed_ = false;
  }

  ros2scope::plugin_api::CoreContext *core_context_{nullptr};
  std::vector<ros2scope::ros_bridge::TopicInfo> topics_;
  std::string selected_topic_;
  std::string selected_type_;
  std::string prepared_type_;

  bool subscribed_{false};
  bool autoscroll_{true};
  bool paused_{false};
  int throttle_ms_{0};
  int decode_throttle_ms_{250};
  int max_array_items_{64};
  int max_messages_{500};

  std::mutex messages_mutex_;
  std::deque<std::string> messages_;
  std::atomic<std::uint64_t> message_counter_{0};
  std::atomic<std::uint64_t> decoded_counter_{0};
  std::chrono::steady_clock::time_point last_emit_time_{};
  std::chrono::steady_clock::time_point last_decode_time_{};
  std::chrono::steady_clock::time_point last_topic_refresh_{};
  std::string last_schema_type_;
  std::string schema_error_;
  std::vector<FieldTreeNode> field_tree_;
  std::string value_error_;
  std::mutex value_mutex_;
  std::vector<ValueTreeNode> value_tree_;

  std::shared_ptr<rcpputils::SharedLibrary> cpp_typesupport_library_;
  std::shared_ptr<rcpputils::SharedLibrary> introspection_typesupport_library_;
  const rosidl_message_type_support_t *cpp_typesupport_{nullptr};
  const rosidl_message_type_support_t *introspection_typesupport_{nullptr};
  std::unique_ptr<rclcpp::SerializationBase> serializer_;
};

}  // namespace

extern "C" std::uint32_t ros2scope_plugin_api_version() {
  return ros2scope::plugin_api::kPluginApiVersion;
}

extern "C" ros2scope::plugin_api::IPlugin *create_plugin() { return new TopicEchoPlugin(); }
