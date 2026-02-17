#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialized_message.hpp>

namespace ros2scope::ros_bridge {

struct TopicInfo {
  std::string name;
  std::string type;
  std::size_t publishers{0};
  std::size_t subscribers{0};
};

class RosBridge {
public:
  using SerializedCallback = std::function<void(const rclcpp::SerializedMessage &)>;

  explicit RosBridge(const std::shared_ptr<rclcpp::Node> &node);

  std::vector<std::string> listNodes() const;
  std::vector<TopicInfo> listTopics() const;

  bool subscribeGeneric(
      const std::string &topic_name,
      const std::string &type_name,
      SerializedCallback callback,
      std::size_t qos_depth = 10);

  void unsubscribe(const std::string &topic_name);

  std::vector<std::string> listParameters(const std::string &node_name) const;
  bool setParameter(const std::string &node_name, const rclcpp::Parameter &parameter) const;

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::unordered_map<std::string, rclcpp::GenericSubscription::SharedPtr> generic_subscriptions_;
};

}  // namespace ros2scope::ros_bridge
