#ifndef PATH_PLANNING_LINEARTRAJECTORY_H
#define PATH_PLANNING_LINEARTRAJECTORY_H

#include "Eigen-3.3/Eigen/Core"
#include <cmath>
#include <map>
#include <vector>

namespace utils {

/**
 * Utility class that interpolates linearly given list of points.
 */
class LinearInterpolator {
 public:
  LinearInterpolator(const std::vector<Eigen::Vector2d>& pts): pts_(pts) {
    if (pts.size() < 2) {
      throw "Atleast 2 points required";
    }
  }

  // Return point at a distance from the first point.
  Eigen::Vector2d PointAtDistance(const double& distance) {
    double len = 0;
    for (int i = 1; i < pts_.size(); ++i) {
      auto segment = pts_[i] - pts_[i-1];
      double seg_len = segment.norm();
      if (len + seg_len >= distance) {
        double pt_dist = distance - len;
        return pts_[i-1] + segment.normalized() * pt_dist;
      }
      len += seg_len;
    }
    auto pt1 = pts_[pts_.size()-2];
    auto pt2 = pts_[pts_.size()-1];
    return pt2 + (pt2 - pt1).normalized() * (distance - len);
  }

 private:
  std::vector<Eigen::Vector2d> pts_;
};

}  // namespace utils

#endif //PATH_PLANNING_LINEARTRAJECTORY_H
