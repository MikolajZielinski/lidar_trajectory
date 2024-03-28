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

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "lidar_trajectory/lidar_trajectory.hpp"

namespace lidar_trajectory
{
using LidarTrajectoryPtr = std::unique_ptr<lidar_trajectory::LidarTrajectory>;

class LIDAR_TRAJECTORY_PUBLIC LidarTrajectoryNode : public rclcpp::Node
{
public:
  explicit LidarTrajectoryNode(const rclcpp::NodeOptions & options);

private:
  LidarTrajectoryPtr lidar_trajectory_{nullptr};
  int64_t param_name_{123};
};
}  // namespace lidar_trajectory

#endif  // LIDAR_TRAJECTORY__LIDAR_TRAJECTORY_NODE_HPP_
