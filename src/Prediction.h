#ifndef PATH_PLANNING_PREDICTION_H
#define PATH_PLANNING_PREDICTION_H

namespace prediction {

/**
 * Prediction module keeps predicts states of other cars for horizon seconds in future.
 */
class Prediction {
 public:
  Prediction(std::shared_ptr <common::Network> network, int horizon = 2)
    : network_(network), horizon_(horizon) {
    timer_.ExecuteAtPeriodicInterval([this]() { Tick(); }, std::chrono::milliseconds(500));
    network->Subscribe<msg::SensorFusion>(
      "sensor_fusion",
      [this](const msg::SensorFusion &sensor_fusion) { UpdateVehicleStates(sensor_fusion); });
  }

 private:
  void Tick() {
    std::lock_guard <std::recursive_mutex> guard(mutex_);
    msg::Prediction prediction;
    for (auto entry : vehicle_states_) {
      prediction.predictions[entry.first] = ComputePrediction(entry.second);
    }
    network_->Publish("prediction", prediction);
  }

  std::vector<msg::VehicleState> ComputePrediction(const msg::VehicleState &state) {
    std::vector <msg::VehicleState> predictions;
    // Constant acceleration model.
    for (int i = 0; i < horizon_; ++i) {
      msg::VehicleState new_state;
      new_state.s_m = fmod(state.s_m + state.velocity_mps * i + 0.5 * state.acceleration_mps2 * i * i, utils::MAX_S_M);
      new_state.velocity_mps = state.velocity_mps;
      new_state.acceleration_mps2 = 0;
      new_state.d_m = state.d_m;
      predictions.emplace_back(new_state);
    }
    return predictions;
  }

  void UpdateVehicleStates(const msg::SensorFusion &sensor_fusion) {
    std::lock_guard <std::recursive_mutex> guard(mutex_);
    vehicle_states_.clear();
    for (auto entry : sensor_fusion.vehicle_states) {
      vehicle_states_[entry.first] = entry.second;
    }
  }

  std::shared_ptr <common::Network> network_;
  int horizon_;
  common::Timer timer_;
  std::map <int, msg::VehicleState> vehicle_states_;
  std::recursive_mutex mutex_;
};

}  // namespace prediction


#endif //PATH_PLANNING_PREDICTION_H
