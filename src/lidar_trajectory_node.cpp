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
  
  subscription_lidar_scan_ = this->create_subscription<LaserScan>(
      "~/input/laser_scan", qos_policy, std::bind(&LidarTrajectoryNode::lidar_scan_callback, this, std::placeholders::_1));
  subscription_odometry_ = this->create_subscription<Odometry>(
      "~/input/current_odometry", qos_policy, std::bind(&LidarTrajectoryNode::odometry_callback, this, std::placeholders::_1));

  publisher_trajectory_ = this->create_publisher<Trajectory>("~/output/trajectory", qos_policy);
  timer_ = this->create_wall_timer(500ms, std::bind(&LidarTrajectoryNode::on_timer, this));

  param_name_ = this->declare_parameter("param_name", 456);
  lidar_trajectory_->foo(param_name_);
}

void LidarTrajectoryNode::lidar_scan_callback(const LaserScan::SharedPtr msg) const
{
  // std::cout << "Reading laser scan" << std::endl;
  this->lidar_trajectory_->laser_scan = std::make_shared<LaserScan>(*msg);
}

void LidarTrajectoryNode::odometry_callback(const Odometry::SharedPtr msg) const
{
  // std::cout << "Reading odometry" << std::endl;
  this->lidar_trajectory_->odometry = std::make_shared<Odometry>(*msg);
}

void LidarTrajectoryNode::on_timer()
{
  // std::cout << "Publish trajectory" << std::endl;
  Trajectory trajectory = this->lidar_trajectory_->calculate_trajectory();

  trajectory.header.stamp = this->now();
  publisher_trajectory_->publish(trajectory);
}

}  // namespace lidar_trajectory

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(lidar_trajectory::LidarTrajectoryNode)
