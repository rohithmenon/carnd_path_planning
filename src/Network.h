#ifndef PATH_PLANNING_NETWORK_H
#define PATH_PLANNING_NETWORK_H

#include <functional>
#include <map>
#include <mutex>

#include "Message.h"

namespace common {

/**
 * A simple network that implements in-memory pubsub.
 */
class Network {
 public:
  template<typename T>
  void Publish(const std::string &topic, const T &message) {
    std::vector<std::function<void(const msg::Message&)>> subscriptions;
    {
      std::lock_guard<std::recursive_mutex> guard(mutex_);
      subscriptions = subscriptions_[topic];
    }
    for (const auto &subscription : subscriptions) {
      subscription(message);
    }
  }

  template<typename T>
  void Subscribe(const std::string &topic, std::function<void(const T &)> subscription) {
    std::lock_guard<std::recursive_mutex> guard(mutex_);
    auto wrapped = [subscription](const msg::Message &msg) {
      subscription(static_cast<const T&>(msg));
    };
    subscriptions_[topic].emplace_back(wrapped);
  }

 private:
  std::map<std::string, std::vector<std::function<void(const msg::Message&)>>> subscriptions_;
  std::recursive_mutex mutex_;
};

}  // namespace common

#endif //PATH_PLANNING_NETWORK_H
