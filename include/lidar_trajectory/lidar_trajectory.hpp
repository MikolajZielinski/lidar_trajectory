// Copyright 2024 Mikolaj Zielinski
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef LIDAR_TRAJECTORY__LIDAR_TRAJECTORY_HPP_
#define LIDAR_TRAJECTORY__LIDAR_TRAJECTORY_HPP_

#include <cstdint>

#include "lidar_trajectory/visibility_control.hpp"

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

namespace lidar_trajectory
{
using sensor_msgs::msg::LaserScan;
using nav_msgs::msg::Odometry;
using geometry_msgs::msg::Quaternion;
using geometry_msgs::msg::Pose;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;

class LIDAR_TRAJECTORY_PUBLIC LidarTrajectory
{
public:
  LidarTrajectory();

  LaserScan::SharedPtr laser_scan;
  Odometry::SharedPtr odometry;

  Trajectory calculate_trajectory(void);
  unsigned long long comb(const unsigned n, const unsigned i);
  std::vector<double> bernstein_poly(int i, int n, const std::vector<double>& t);
  std::vector<std::vector<double>> bezier_curve(const std::vector<std::vector<double>>& points, const int nTimes);

  int64_t foo(int64_t bar) const;
};
}  // namespace lidar_trajectory

#endif  // LIDAR_TRAJECTORY__LIDAR_TRAJECTORY_HPP_
