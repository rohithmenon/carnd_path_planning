#ifndef PATH_PLANNING_LOCALIZATION_H
#define PATH_PLANNING_LOCALIZATION_H

#include <memory>
#include <mutex>

#include "Message.h"
#include "Network.h"
#include "Utils.h"

namespace localization {

/**
 * Class responsible for localizing the main vehicle. Given we get true date about vehicle position,
 * this class just wraps data available into expected structures.
 */
class Localization {
 public:
  Localization(std::shared_ptr <common::Network> network)
    : network_(network),
      accel_timestamp_(0),
      last_velocity_mps_(0) {

    network->Subscribe<msg::Localization>(
      "localization",
      [this](const msg::Localization &localization) { UpdateLocalization(localization); });
  }

 private:
  
  void UpdateLocalization(const msg::Localization &localization) {
    std::lock_guard <std::recursive_mutex> guard(mutex_);
    state_data_.d_m = localization.d_m;
    state_data_.s_m = localization.s_m;
    double velocity_mps = utils::MilesPerHourToMetersPerSecond(localization.velocity_mph);
    if (accel_timestamp_ == 0) {
      accel_timestamp_ = localization.millis;
      last_velocity_mps_ = velocity_mps;
    }
    if (localization.millis > accel_timestamp_ + 1000) {
      state_data_.acceleration_mps2 = (velocity_mps - last_velocity_mps_) / (localization.millis - accel_timestamp_);
      accel_timestamp_ = localization.millis;
      last_velocity_mps_ = velocity_mps;
    }
    state_data_.velocity_mps = velocity_mps;
    state_data_.yaw_rad = localization.yaw_rad;

    network_->Publish("localized_state", state_data_);
  }

  std::shared_ptr <common::Network> network_;
  msg::VehicleState state_data_;
  std::recursive_mutex mutex_;
  long accel_timestamp_;
  double last_velocity_mps_;
};

}  // namespace localization

#endif //PATH_PLANNING_LOCALIZATION_H
