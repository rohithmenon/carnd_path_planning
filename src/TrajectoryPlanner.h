#ifndef PATH_PLANNING_TRAJECTORYPLANNER_H
#define PATH_PLANNING_TRAJECTORYPLANNER_H

#include <complex>
#include <math.h>
#include <memory>
#include <mutex>

#include "Eigen-3.3/Eigen/Core"
#include "Clock.h"
#include "Message.h"
#include "Network.h"
#include "Timer.h"
#include "TrajectoryPlanner.h"
#include "LinearInterpolator.h"
#include "Spline.h"

namespace trajectory_planner {

// A simple trajectory planner that plans trajectory to a target within the constraints imposed.
class TrajectoryPlanner {
 public:
  TrajectoryPlanner(std::shared_ptr<common::Network> network,
                    utils::CoordConversions& coord_conversions,
                    double speed_limit_mph,
                    double max_accel_mps2,
                    double max_jerk_mps3,
                    size_t trajectory_size,
                    double resolution_secs)
      : network_(network),
        coord_conversions_(coord_conversions),
        speed_limit_mps_(utils::MilesPerHourToMetersPerSecond(speed_limit_mph)),
        max_accel_mps2_(max_accel_mps2),
        max_jerk_mps3_(max_jerk_mps3),
        trajectory_size_(trajectory_size),
        resolution_secs_(resolution_secs) {
    behaviour_cmd_.target_state.s_m = -1;
    behaviour_cmd_.target_state.d_m = 6;
    behaviour_cmd_.target_state.velocity_mps = 0;
    behaviour_cmd_.target_state.acceleration_mps2 = 0;
    network->Subscribe<msg::BehaviourCommand>(
      "vehicle_behaviour_cmd",
      [this](const msg::BehaviourCommand &behaviour_cmd) {
        std::lock_guard<std::recursive_mutex> guard(mutex_);
        behaviour_cmd_ = behaviour_cmd;
      });

    network->Subscribe<msg::VehicleState>(
      "localized_state",
      [this](const msg::VehicleState& vehicle_state) {
        std::lock_guard<std::recursive_mutex> guard(mutex_);
        state_data_ = vehicle_state;
      });
  }

  // Compute new trajectory for the car given previous trajectory.
  std::vector<msg::WayPointXY> GetTrajectory(const std::vector<msg::WayPointXY> &previous_trajectory) {
    std::lock_guard<std::recursive_mutex> guard(mutex_);
    std::vector<msg::WayPointXY> ret;
    if (behaviour_cmd_.target_state.s_m < 0) {
      return ret;
    }

    Eigen::Vector2d cur_pos;
    auto cur_xy = coord_conversions_.getXYSpline(state_data_.s_m, state_data_.d_m);
    cur_pos << cur_xy[0], cur_xy[1];

    double traj_distance = 0;
    Eigen::Vector2d prev_pos = cur_pos;

    // Use previous trajectory points.
    for (const auto &wpt : previous_trajectory) {
      Eigen::Vector2d wpt_vec2 = wpt.ToEigen();
      traj_distance += (wpt_vec2 - prev_pos).norm();
      prev_pos = wpt_vec2;
      ret.emplace_back(wpt);
    }

    // Compute new points based on input from beheviour command.
    std::vector<Eigen::Vector2d> new_pts;
    new_pts.emplace_back(prev_pos);
    auto prev_fc = coord_conversions_.getFrenet(prev_pos[0], prev_pos[1], state_data_.yaw_rad);
    for (int i = 0; i < 5; ++i) {
      auto target_s = fmod(state_data_.s_m + (i + 1) * 30, utils::MAX_S_M);
      auto pt_dist = utils::FabsDiffS(target_s, state_data_.s_m);
      if (pt_dist > traj_distance) {
        auto delta_d = behaviour_cmd_.target_state.d_m > prev_fc[1] ? 1.0 : -1.0;
        auto min_d = std::min(behaviour_cmd_.target_state.d_m, prev_fc[1]);
        auto max_d = std::max(behaviour_cmd_.target_state.d_m, prev_fc[1]);
        auto target_d = std::clamp(prev_fc[1] + (i + 1) * delta_d, min_d, max_d);
        auto xy = coord_conversions_.getXYSpline(target_s, target_d);
        Eigen::Vector2d pt({xy[0], xy[1]});
        new_pts.emplace_back(pt);
      }
    }

    if (new_pts.size() < 2) {
      return ret;
    }

    // Linearly interpolate the points.
    utils::LinearInterpolator interp(new_pts);

    long ret_sz = ret.size();
    double s = 0, v = 0;
    if (ret_sz > 2) {
      auto s0 = ret[ret_sz - 3];
      auto s1 = ret[ret_sz - 2];
      auto s2 = ret[ret_sz - 1];
      v = (s2.ToEigen() - s1.ToEigen()).norm();
    }
    double vi = v;
    double vf = behaviour_cmd_.target_state.velocity_mps * resolution_secs_;

    // Compute new points that follow previous trajectory points.
    ulong num_pts = ret.size();
    while (num_pts < trajectory_size_) {
      double dir = vf > v ? 1.0 : -1.0;

      auto min_v = dir > 0 ? vi : vf;
      auto max_v = dir > 0 ? vf : vi;
      v = std::clamp(v + dir * 0.001, min_v, max_v);
      s = s + v;

      auto pt = interp.PointAtDistance(s);
      msg::WayPointXY waypt;
      waypt.x_m = pt(0);
      waypt.y_m = pt(1);
      ret.emplace_back(waypt);
      num_pts++;
    }

    // Smooth the trajectory above using spline.
    return Smooth(ret);
  }

 private:
  // Smooth a given set of waypoints using spline.
  std::vector<msg::WayPointXY> Smooth(const std::vector<msg::WayPointXY> wpts) {
    std::vector<Eigen::Vector2d> pts;
    for (const auto& wpt : wpts) {
      pts.emplace_back(wpt.ToEigen());
    }

    // Shift and rotate the given trajectory around first waypoint and in the direction
    // of the second point.
    Eigen::Vector2d dir_vec = pts[1] - pts[0];
    double rad = atan2(dir_vec(1), dir_vec(0));
    Eigen::MatrixXd rot_mat(2, 2), rot_mat_inv(2, 2);
    rot_mat << cos(-rad), -sin(-rad),
      sin(-rad), cos(-rad);
    rot_mat_inv << cos(rad), -sin(rad),
      sin(rad), cos(rad);
    std::vector<Eigen::Vector2d> tr;
    for (const auto& pt : pts) {
      tr.emplace_back(rot_mat * (pt - pts[0]));
    }

    // Create a spline with first 10 points and last two points of the trajectory
    int num_pts = pts.size();
    std::vector<double> x_vals, y_vals;
    for (int i = 0; i < 10 && i < num_pts - 2; ++i) {
      x_vals.emplace_back(tr[i][0]);
      y_vals.emplace_back(tr[i][1]);
    }
    x_vals.emplace_back(tr[num_pts - 1][0]);
    y_vals.emplace_back(tr[num_pts - 1][1]);

    // Smooth using spline
    tk::spline spline;
    spline.set_points(x_vals, y_vals);
    std::vector<msg::WayPointXY> smoothed;
    for (int i = 0; i < num_pts; ++i) {
      Eigen::Vector2d new_pt;
      new_pt << tr[i][0], spline(tr[i][0]);
      auto tr_inv = (rot_mat_inv * new_pt) + pts[0];
      msg::WayPointXY wpt;
      wpt.x_m = tr_inv[0];
      wpt.y_m = tr_inv[1];
      smoothed.emplace_back(wpt);
    }
    return smoothed;
  } 

  std::shared_ptr<common::Network> network_;
  utils::CoordConversions& coord_conversions_;
  double speed_limit_mps_;
  double max_accel_mps2_;
  double max_jerk_mps3_;
  ulong trajectory_size_;
  double resolution_secs_;
  msg::VehicleState state_data_;
  msg::BehaviourCommand behaviour_cmd_;
  std::recursive_mutex mutex_;
};

}  // namespace trajectory_planner

#endif //PATH_PLANNING_TRAJECTORYPLANNER_H
