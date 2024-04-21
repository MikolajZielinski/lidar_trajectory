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

#ifndef LIDAR_TRAJECTORY__LIDAR_TRAJECTORY_NODE_HPP_
#define LIDAR_TRAJECTORY__LIDAR_TRAJECTORY_NODE_HPP_

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "lidar_trajectory/lidar_trajectory.hpp"

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace lidar_trajectory
{
using namespace std::chrono_literals;
using LidarTrajectoryPtr = std::unique_ptr<lidar_trajectory::LidarTrajectory>;
using sensor_msgs::msg::LaserScan;
using nav_msgs::msg::Odometry;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;

class LIDAR_TRAJECTORY_PUBLIC LidarTrajectoryNode : public rclcpp::Node
{
public:
  explicit LidarTrajectoryNode(const rclcpp::NodeOptions & options);

private:
  LidarTrajectoryPtr lidar_trajectory_{nullptr};
  int64_t param_name_{123};

  rclcpp::Subscription<LaserScan>::SharedPtr subscription_lidar_scan_;
  rclcpp::Subscription<Odometry>::SharedPtr subscription_odometry_;

  rclcpp::Publisher<Trajectory>::SharedPtr publisher_trajectory_;
  rclcpp::TimerBase::SharedPtr timer_;

  void lidar_scan_callback(const LaserScan::SharedPtr msg) const;
  void odometry_callback(const Odometry::SharedPtr msg) const;
  void on_timer();
};
}  // namespace lidar_trajectory

#endif  // LIDAR_TRAJECTORY__LIDAR_TRAJECTORY_NODE_HPP_
