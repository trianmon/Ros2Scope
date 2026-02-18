// SPDX-FileCopyrightText: 2026 Georgy Grigoriev (GitHub: trianmon)
// SPDX-License-Identifier: Apache-2.0

#include "ros2scope/ros_bridge/ros_bridge.hpp"

#include <algorithm>

#include <rcl_interfaces/srv/list_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>

namespace ros2scope::ros_bridge {

RosBridge::RosBridge(const std::shared_ptr<rclcpp::Node> &node) : node_(node) {}

std::vector<std::string> RosBridge::listNodes() const {
  std::vector<std::string> output;
  auto names_and_namespaces = node_->get_node_graph_interface()->get_node_names_and_namespaces();
  output.reserve(names_and_namespaces.size());
  for (const auto &entry : names_and_namespaces) {
    output.push_back(entry.first);
  }
  std::sort(output.begin(), output.end());
  return output;
}

std::vector<TopicInfo> RosBridge::listTopics() const {
  std::vector<TopicInfo> output;
  const auto topics = node_->get_topic_names_and_types();
  output.reserve(topics.size());

  for (const auto &[topic_name, types] : topics) {
    TopicInfo info;
    info.name = topic_name;
    info.type = types.empty() ? std::string{} : types.front();

    std::size_t publishers = 0;
    std::size_t subscribers = 0;

    for (const auto &name : node_->get_publishers_info_by_topic(topic_name)) {
      (void)name;
      ++publishers;
    }
    for (const auto &name : node_->get_subscriptions_info_by_topic(topic_name)) {
      (void)name;
      ++subscribers;
    }

    info.publishers = publishers;
    info.subscribers = subscribers;
    output.push_back(std::move(info));
  }

  std::sort(output.begin(), output.end(), [](const auto &lhs, const auto &rhs) {
    return lhs.name < rhs.name;
  });

  return output;
}

bool RosBridge::subscribeGeneric(
    const std::string &topic_name,
    const std::string &type_name,
    SerializedCallback callback,
    std::size_t qos_depth) {
  if (topic_name.empty() || type_name.empty()) {
    return false;
  }

  rclcpp::QoS qos{rclcpp::KeepLast(qos_depth)};
  auto subscription = node_->create_generic_subscription(
      topic_name,
      type_name,
      qos,
      [cb = std::move(callback)](std::shared_ptr<rclcpp::SerializedMessage> msg) {
        if (msg != nullptr) {
          cb(*msg);
        }
      });

  generic_subscriptions_[topic_name] = std::move(subscription);
  return true;
}

void RosBridge::unsubscribe(const std::string &topic_name) {
  generic_subscriptions_.erase(topic_name);
}

std::vector<std::string> RosBridge::listParameters(const std::string &node_name) const {
  const auto service_name = node_name + "/list_parameters";
  auto client = node_->create_client<rcl_interfaces::srv::ListParameters>(service_name);
  if (!client->wait_for_service(std::chrono::milliseconds(200))) {
    return {};
  }

  auto request = std::make_shared<rcl_interfaces::srv::ListParameters::Request>();
  request->depth = 100;
  auto response_future = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, response_future, std::chrono::milliseconds(300)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    return {};
  }

  return response_future.get()->result.names;
}

bool RosBridge::setParameter(const std::string &node_name, const rclcpp::Parameter &parameter) const {
  const auto service_name = node_name + "/set_parameters";
  auto client = node_->create_client<rcl_interfaces::srv::SetParameters>(service_name);
  if (!client->wait_for_service(std::chrono::milliseconds(200))) {
    return false;
  }

  auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
  request->parameters.push_back(parameter.to_parameter_msg());
  auto response_future = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, response_future, std::chrono::milliseconds(300)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    return false;
  }

  const auto &response = response_future.get();
  return !response->results.empty() && response->results.front().successful;
}

}  // namespace ros2scope::ros_bridge
