#pragma once

#include <any>
#include <cstddef>
#include <functional>
#include <mutex>
#include <string>
#include <unordered_map>

namespace ros2scope::event_bus {

class EventBus {
public:
  using HandlerId = std::size_t;
  using Handler = std::function<void(const std::any &)>;

  HandlerId subscribe(const std::string &topic, Handler handler) {
    std::scoped_lock lock(mutex_);
    const HandlerId id = ++last_id_;
    handlers_[topic][id] = std::move(handler);
    return id;
  }

  void unsubscribe(const std::string &topic, HandlerId handler_id) {
    std::scoped_lock lock(mutex_);
    auto topic_it = handlers_.find(topic);
    if (topic_it == handlers_.end()) {
      return;
    }
    topic_it->second.erase(handler_id);
    if (topic_it->second.empty()) {
      handlers_.erase(topic_it);
    }
  }

  template <typename T>
  void publish(const std::string &topic, T payload) {
    publishAny(topic, std::any{std::move(payload)});
  }

  void publishAny(const std::string &topic, std::any payload) {
    std::unordered_map<HandlerId, Handler> handlers_copy;
    {
      std::scoped_lock lock(mutex_);
      auto topic_it = handlers_.find(topic);
      if (topic_it == handlers_.end()) {
        return;
      }
      handlers_copy = topic_it->second;
    }

    for (const auto &[_, handler] : handlers_copy) {
      handler(payload);
    }
  }

private:
  std::mutex mutex_;
  HandlerId last_id_{0};
  std::unordered_map<std::string, std::unordered_map<HandlerId, Handler>> handlers_;
};

}  // namespace ros2scope::event_bus
