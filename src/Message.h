#ifndef PATH_PLANNING_MESSAGE_H
#define PATH_PLANNING_MESSAGE_H

#include "Eigen-3.3/Eigen/Core"
#include <map>
#include <string>
#include <vector>
#include <vector>

namespace msg {

/// \brief Base class for all messages
class Message {
 public:
  Message() {}
  virtual ~Message() {}
  virtual std::string ToString() const = 0;
};

struct WayPointXY : public Message {
  double x_m;
  double y_m;

  Eigen::Vector2d ToEigen() const {
    Eigen::Vector2d vec2;
    vec2 << x_m, y_m;
    return vec2;
  }

  std::string ToString()  const override {
    std::stringstream ss;
    ss << "WayPointXY: [x_m: " << x_m
       << ", y_m: " << y_m << "]";
    return ss.str();
  }
};

struct VehicleState : public Message {
  double d_m;
  double s_m;
  double velocity_mps;
  double acceleration_mps2;
  double yaw_rad;

  std::string ToString()  const override {
    std::stringstream ss;
    ss << "VehicleState: [d_m: " << d_m
       << ", s_m: " << s_m
       << ", velocity_mps: " << velocity_mps
       << ", acceleration_mps2: " << acceleration_mps2 << "]";
    return ss.str();
  }
};

struct Localization : public Message {
  double s_m;
  double d_m;
  double x_m;
  double y_m;
  double yaw_rad;
  double velocity_mph;
  long millis;

  std::string ToString()  const override {
    std::stringstream ss;
    ss << "Localization: [s_m: " << s_m
       << ", d_m: " << d_m
       << ", x_m: " << x_m
       << ", y_m: " << y_m
       << ", yaw_rad: " << yaw_rad
       << ", velocity_mph: " << velocity_mph
       << ", millis: " << millis << "]";
    return ss.str();
  }
};

struct SensorFusion : public Message {
  std::map<int, VehicleState> vehicle_states;

  std::string ToString()  const override {
    std::stringstream ss;
    ss << "SensorFusion: [" << std::endl;
    for (auto entry : vehicle_states) {
      ss << entry.first << ": " << entry.second.ToString() << std::endl;
    }
    ss << "]" << std::endl;
    return ss.str();
  }
};

struct Prediction : public Message {
  std::map<int, std::vector<VehicleState>> predictions;

  std::string ToString()  const override {
    std::stringstream ss;
    ss << "Prediction: [" << std::endl;
    for (auto entry : predictions) {
      ss << entry.first << ": [";
      for (auto state : entry.second) {
        ss << state.ToString() << std::endl;
      }
      ss << "]" << std::endl;
    }
    ss << "]" << std::endl;
    return ss.str();
  }
};

struct BehaviourCommand : public Message {
  long timestamp_millis;
  VehicleState target_state;

  std::string ToString()  const override {
    std::stringstream ss;
    ss << "BehaviourCommand: [timestamp_millis: " << timestamp_millis
       << ", target: " << target_state.ToString() << "]" << std::endl;
    return ss.str();
  }
};

}  // namespace common

#endif //PATH_PLANNING_MESSAGE_H
