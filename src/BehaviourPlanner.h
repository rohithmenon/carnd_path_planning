#ifndef PATH_PLANNING_BEHAVIOURPLANNER_H
#define PATH_PLANNING_BEHAVIOURPLANNER_H

#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

#include "Clock.h"
#include "Message.h"
#include "Network.h"
#include "Timer.h"
#include "Utils.h"

namespace behaviour_planner {

enum State {
  KeepLane = 0,
  LaneChangeLeft = 1,
  LaneChangeRight = 2
};

/**
 * Behaviour planner. Responsibilities of the class includes avoiding collision,
 * maintaining safe distance and passing.
 */
class BehaviourPlanner {
 public:
    explicit BehaviourPlanner(std::shared_ptr<common::Network> network,
                              double max_acceleration_mps2,
                              double max_jerk_mps3,
                              double speed_limit_mph,
                              double vehicle_buffer_fwd_m,
                              double vehicle_buffer_beh_m,
                              int num_lanes,
                              int behaviour_horizon_secs)
      : network_(network),
        state_(KeepLane),
        target_lane_(1),
        max_acceleration_mps2_(max_acceleration_mps2),
        max_jerk_mps3_(max_jerk_mps3),
        speed_limit_mps_(utils::MilesPerHourToMetersPerSecond(speed_limit_mph)),
        vehicle_buffer_fwd_m_(vehicle_buffer_fwd_m),
        vehicle_buffer_beh_m_(vehicle_buffer_beh_m),
        num_lanes_(num_lanes),
        behaviour_horizon_secs_(behaviour_horizon_secs) {
    state_data_.velocity_mps = 0.0;
    state_data_.s_m = -1;
    state_data_.d_m = utils::GetD(1);
    state_data_.acceleration_mps2 = 0.0;

    timer_.ExecuteAtPeriodicInterval([this]() { Tick(); }, std::chrono::milliseconds(1000));
    network->Subscribe<msg::Prediction>(
      "prediction",
      [this](const msg::Prediction& predictions) { UpdatePredictions(predictions); });
    network->Subscribe<msg::VehicleState>(
      "localized_state",
      [this](const msg::VehicleState& vehicle_state) {
        std::lock_guard<std::recursive_mutex> guard(mutex_);
        if (state_data_.s_m < 0) {
          target_lane_ = utils::GetLane(vehicle_state.d_m);
        }
        state_data_ = vehicle_state;
      });
  }

 private:
  void Tick() {
    std::lock_guard<std::recursive_mutex> guard(mutex_);
    if (state_data_.s_m < 0) {
      return;
    }

    // Get valid next states.
    std::vector<State> next_states = NextStates();
    double min_cost = std::numeric_limits<double>::max();
    State best_next_state = state_;
    std::optional<msg::VehicleState> best_next_vehicle_state;

    for (const auto& next_state : next_states) {
      // Compute state after transition to next_state.
      std::optional<msg::VehicleState> next_vehicle_state = Transition(next_state);
      if (!next_vehicle_state) {
        continue;
      }
      // Compute cost of transition.
      double cost = ComputeCost(next_state, *next_vehicle_state);
      if (cost < min_cost) {
        min_cost = cost;
        best_next_state = next_state;
        best_next_vehicle_state.emplace(*next_vehicle_state);
      }
    }

    // Choose the best next vehicle state.
    if (best_next_vehicle_state) {
      state_ = best_next_state;
      if (state_ == LaneChangeLeft) {
        target_lane_ = std::clamp(target_lane_ - 1, 0, num_lanes_ - 1);
      } else if (state_ == LaneChangeRight) {
        target_lane_ = std::clamp(target_lane_ + 1, 0, num_lanes_ - 1);
      }
      msg::BehaviourCommand cmd;
      cmd.target_state = *best_next_vehicle_state;
      cmd.timestamp_millis = common::Clock::NowMillis() + behaviour_horizon_secs_ * 1000;
      network_->Publish("vehicle_behaviour_cmd", cmd);
      std::cout << state_data_.ToString() << std::endl;
      LOG << cmd.ToString() << std::endl;
    }
  }

  // Compute next states from current states.
  std::vector<State> NextStates() {
    std::vector<State> next_states;
    switch (state_) {
      case KeepLane:
        next_states.emplace_back(KeepLane);
        if (utils::GetLane(state_data_.d_m) > 0) {
          next_states.emplace_back(LaneChangeLeft);
        }
        if (utils::GetLane(state_data_.d_m) < num_lanes_ - 1) {
          next_states.emplace_back(LaneChangeRight);
        }
        break;
      case LaneChangeLeft:
      case LaneChangeRight:
        next_states.emplace_back(KeepLane);
        break;
    }
    return next_states;
  }

  // Compute cost of next state given current state.
  double ComputeCost(const State& next_state, const msg::VehicleState& next_state_data) {
    return 1.0 / std::max(0.001, next_state_data.velocity_mps - state_data_.velocity_mps);
  }

  // Compute next state data given next state
  std::optional<msg::VehicleState> Transition(const State& next_state) {
    switch (next_state) {
      case KeepLane:
        return KeepLaneTransition();
      case LaneChangeLeft:
        return LaneChangeTransition(true);
      case LaneChangeRight:
        return LaneChangeTransition(false);
    }
  }

  // State data for maintaining current lane.
  std::optional<msg::VehicleState> KeepLaneTransition() {
    return GetVehicleStateForLane(target_lane_);
  }

  // Get State data for a left/right lane change.
  std::optional<msg::VehicleState> LaneChangeTransition(bool left) {
    int cur_lane = utils::GetLane(state_data_.d_m);
    int next_lane = std::clamp(cur_lane + (left ? -1 : 1), 0, num_lanes_ - 1);
    for (auto entry : predictions_) {
      const std::vector<msg::VehicleState> vehicle_predictions = entry.second;
      // Only transition if the next lane is safe given buffer distances
      if (!vehicle_predictions.empty()) {
        const auto& vehicle_prediction = vehicle_predictions[0];
        if (next_lane == utils::GetLane(vehicle_prediction.d_m)
            && ((state_data_.s_m <= vehicle_prediction.s_m && state_data_.s_m + vehicle_buffer_fwd_m_ >= vehicle_prediction.s_m)
                 || (state_data_.s_m >= vehicle_prediction.s_m && state_data_.s_m - vehicle_buffer_beh_m_ <= vehicle_prediction.s_m))) {
          return {};
        }
      }
    }
    return GetVehicleStateForLane(next_lane);
  }

  // Get State data for a given lane.
  msg::VehicleState GetVehicleStateForLane(int lane) {
    msg::VehicleState state_for_lane;
    state_for_lane.d_m = utils::GetD(lane);

    std::optional<msg::VehicleState> ahead = GetVehicleAhead(lane);
    double cur_velocity = state_data_.velocity_mps;
    double target_velocity = speed_limit_mps_;

    if (ahead && state_data_.s_m + vehicle_buffer_fwd_m_ > ahead->s_m) {
      target_velocity = ahead->velocity_mps - 1;
    }

    double max_vel_accel_limit_mps2 = max_acceleration_mps2_ * behaviour_horizon_secs_ + state_data_.velocity_mps;
    target_velocity = std::min(max_vel_accel_limit_mps2, target_velocity);
    double target_accel = (target_velocity - cur_velocity) / behaviour_horizon_secs_;

    state_for_lane.velocity_mps = target_velocity;
    state_for_lane.acceleration_mps2 = target_accel;
    state_for_lane.s_m = state_data_.s_m
      + state_data_.velocity_mps * behaviour_horizon_secs_
      + 0.5 * state_for_lane.acceleration_mps2 * behaviour_horizon_secs_ * behaviour_horizon_secs_;
    return state_for_lane;
  }

  // Get the state of the vehicle immediately ahead of current vehicle in the lane queried.
  std::optional<msg::VehicleState> GetVehicleAhead(int lane) {
    std::optional<msg::VehicleState> ahead = std::nullopt;
    for (const auto& entry : predictions_) {
      const auto& vehicle_predictions = entry.second;
      if (!vehicle_predictions.empty()) {
        const auto vehicle_prediction = vehicle_predictions[0];
        if (utils::GetLane(vehicle_prediction.d_m) == lane
            && utils::FDiffS(state_data_.s_m, vehicle_prediction.s_m) > 0) {
          if (!ahead) {
            ahead.emplace(vehicle_prediction);
          } else {
            if (utils::FabsDiffS(ahead->s_m, state_data_.s_m) > utils::FabsDiffS(vehicle_prediction.s_m, state_data_.s_m)) {
              ahead.emplace(vehicle_prediction);
            }
          }
        }
      }
    }
    return ahead;
  }

  // Update predictions about other cars.
  void UpdatePredictions(const msg::Prediction& predictions) {
    std::lock_guard<std::recursive_mutex> guard(mutex_);
    for (auto entry : predictions.predictions) {
      predictions_[entry.first] = entry.second;
    }
  }

  std::shared_ptr<common::Network> network_;
  common::Timer timer_;
  State state_;
  int target_lane_;
  msg::VehicleState state_data_;
  std::map<int, std::vector<msg::VehicleState>> predictions_;
  std::recursive_mutex mutex_;
  double max_acceleration_mps2_;
  double max_jerk_mps3_;
  double speed_limit_mps_;
  double vehicle_buffer_fwd_m_;
  double vehicle_buffer_beh_m_;
  int num_lanes_;
  int behaviour_horizon_secs_;
};

}  // namespace behaviour_planner

#endif //PATH_PLANNING_BEHAVIOURPLANNER_H
