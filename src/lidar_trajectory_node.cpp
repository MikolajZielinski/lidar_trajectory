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

#include "lidar_trajectory/lidar_trajectory_node.hpp"

namespace lidar_trajectory
{

LidarTrajectoryNode::LidarTrajectoryNode(const rclcpp::NodeOptions & options)
:  Node("lidar_trajectory", options)
{
  lidar_trajectory_ = std::make_unique<lidar_trajectory::LidarTrajectory>();
  auto qos_policy = rclcpp::QoS(1);
  qos_policy.best_effort();
  subscription_lidar_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "~/input/laser_scan", qos_policy, std::bind(&LidarTrajectoryNode::lidar_scan_callback, this, std::placeholders::_1));
  param_name_ = this->declare_parameter("param_name", 456);
  lidar_trajectory_->foo(param_name_);
}

void LidarTrajectoryNode::lidar_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const
{
  sensor_msgs::msg::LaserScan::SharedPtr a = msg;
  // std::cout << "Reading laser scan" << std::endl;
}

}  // namespace lidar_trajectory

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(lidar_trajectory::LidarTrajectoryNode)
