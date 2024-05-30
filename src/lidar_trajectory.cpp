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

#include "lidar_trajectory/lidar_trajectory.hpp"

#include <math.h>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <vector>

namespace lidar_trajectory
{

LidarTrajectory::LidarTrajectory()
{
  laser_scan = nullptr;
  odometry = nullptr;
}

int64_t LidarTrajectory::foo(int64_t bar) const
{
  std::cout << "Hello World, " << bar << std::endl; 
  return bar;
}

Trajectory LidarTrajectory::calculate_trajectory(void)
{
  Trajectory trajectory;

  if(this->laser_scan != nullptr && this->odometry != nullptr)
  {
    // Convert quaternion to RPY
    const Odometry odom = *this->odometry;
    Quaternion quat = odom.pose.pose.orientation;
    tf2::Quaternion tf_quaternion;
    tf2::fromMsg(quat, tf_quaternion);
    tf2::Matrix3x3 rotation_matrix(tf_quaternion);

    double roll, pitch, start_angle;
    rotation_matrix.getRPY(roll, pitch, start_angle);

    start_angle = start_angle - (M_PI / 2);

    std::vector<double> start_direction = {sin(-start_angle), cos(start_angle)};
    std::vector<double> start_point = {odom.pose.pose.position.x, -odom.pose.pose.position.y};

    // Read LaserScan data
    const LaserScan laser = *this->laser_scan;
    std::vector<double> distances; // TODO remove outliers
    for(double range : laser.ranges)
    {
      distances.push_back(range);
    }
    int num_points = int(distances.size());
    double scan_angle = laser.angle_increment;
    double offset_angle = (((M_PI * 2) / scan_angle) - num_points) / 2;
    // double max_dist = laser.range_max;
    double max_dist = 5.0;

    // Convert lidar distances to x, y coordinates
    std::vector<std::vector<double>> coords;
    for(int i=0; i<num_points; i++)
    {
      double dist = distances[i];

      if(dist < max_dist - 0.05)
      {
        double x = (sin((scan_angle * (i + offset_angle)) + start_angle) * dist) + start_point[0];
        double y = (cos((scan_angle * (i + offset_angle)) + start_angle) * dist) + start_point[1];

        std::vector<double> point = {x, y};

        coords.push_back(point);
      }
    }

    // Filter coords
    std::vector<std::vector<double>> filtered_coords;
    for(int i=1; i<int(coords.size() - 3); i++)
    {
      int j = i + 1;
      int k = j + 1;

      double dist_1 = std::hypot(coords[i][0] - coords[j][0], coords[i][1] - coords[j][1]);
      double dist_2 = std::hypot(coords[j][0] - coords[k][0], coords[j][1] - coords[k][1]);
      double dist_3 = std::hypot(coords[i][0] - coords[k][0], coords[i][1] - coords[k][1]);

      if(dist_3 < dist_1 && dist_3 < dist_2 && (dist_1 > 0.05 || dist_2 > 0.05))
      {
        continue;
      }

      filtered_coords.push_back(coords[i]);
    } 
    coords = filtered_coords;

    class LineSegment
    {
      public:
        std::vector<double> point;
        double value;
    };

    // Create point sections 
    std::vector<std::vector<LineSegment>> line_sections;
    std::vector<LineSegment> line_subsection;
    float threshold = 0.8;
    for(int i=0; i<int(coords.size()); i++)
    {
      int j = i + 1;

      if(j == int(coords.size()))
      {
        if(int(line_subsection.size()) > 10)
        {
          line_sections.push_back(line_subsection);
        }
        break;
      }

      double points_dist = std::hypot(coords[i][0] - coords[j][0], coords[i][1] - coords[j][1]);
      if(points_dist < threshold)
      {
        LineSegment segment;
        segment.point = coords[i];
        segment.value = points_dist;
        line_subsection.push_back(segment);
      }
      else
      {
        if (int(line_subsection.size()) > 10)
        {
          line_sections.push_back(line_subsection);
        }
        line_subsection.clear();
      }
    }

    // Reduce number of points on a section
    std::vector<std::vector<std::vector<double>>> line_sections_reduced;
    for(std::vector<LineSegment> line_sub : line_sections)
    {
      double distance = 0;
      std::vector<std::vector<double>> line_subsection_reduced;
      for(LineSegment line_seg : line_sub)
      {
        distance += line_seg.value;

        if(distance >= 0.3)
        {
          distance = 0;
          line_subsection_reduced.push_back(line_seg.point);
        }
      }

      if(int(line_subsection_reduced.size()) > 1)
      {
        line_subsection_reduced.insert(line_subsection_reduced.begin(), line_sub[0].point);
        line_sections_reduced.push_back(line_subsection_reduced);
      }
    }

    // Print sections lengths
    int k = 0;
    for(auto sec : line_sections)
    {
      std::cout << k << ". " << int(sec.size()) << std::endl;
      k++;
    }
    std::cout << "#########################################################################################" << std::endl;

    // Calculate normal vectors
    float distance_from_border = 0.8;
    std::vector<std::vector<std::vector<double>>> normal_vectors_sections;
    for(std::vector<std::vector<double>> line_sub_red : line_sections_reduced)
    {
      std::vector<std::vector<double>> normal_vectors_subsections;
      for(int i=0; i<int(line_sub_red.size() - 1); i++)
      {
        double x1 = line_sub_red[i][0];
        double y1 = line_sub_red[i][1];
        double x2 = line_sub_red[i + 1][0];
        double y2 = line_sub_red[i + 1][1];

        double nx = (y2 - y1);
        double ny = -(x2 - x1);

        double n_x = nx / std::hypot(nx, ny) * distance_from_border;
        double n_y = ny / std::hypot(nx, ny) * distance_from_border;

        std::vector<double> point = {n_x + x1, n_y + y1};
        normal_vectors_subsections.push_back(point);
      }
      normal_vectors_sections.push_back(normal_vectors_subsections);
    }

    // Check if normal vectors are intersecting each other and remove them
    std::vector<std::vector<std::vector<double>>> no_inter_sections;
    for(int i=0; i<int(line_sections_reduced.size()); i++)
    {
      std::vector<std::vector<double>> line_sub_red = line_sections_reduced[i];
      std::vector<std::vector<double>> norm_sub = normal_vectors_sections[i];
      std::vector<std::vector<double>> no_inter_subsections;

      for(int j=0; j<int(norm_sub.size() - 1); j++)
      {
        double x1 = line_sub_red[j][0];
        double y1 = line_sub_red[j][1];
        double x2 = norm_sub[j][0];
        double y2 = norm_sub[j][1];
        double x3 = line_sub_red[j + 1][0];
        double y3 = line_sub_red[j + 1][1];
        double x4 = norm_sub[j + 1][0];
        double y4 = norm_sub[j + 1][1];

        if(((x1 < x3 && x2 > x4) || (x1 > x3 && x2 < x4)) || ((y1 < y3 && y2 > y4) || (y1 > y3 && y2 < y4)))
        {

        }
        else
        {
          no_inter_subsections.push_back(norm_sub[j]);
        }
      }
      no_inter_subsections.push_back(norm_sub.back());

      if(int(no_inter_subsections.size()) > 1)
      {
        no_inter_sections.push_back(no_inter_subsections);
      }
    }

    // Connect edges
    double x0 = start_point[0];
    double y0 = start_point[1];
    std::vector<LineSegment> perimeter_points;
    for(int i=0; i<int(no_inter_sections.size()); i++)
    {
      std::vector<std::vector<double>> no_inter_sub = no_inter_sections[i];
      double x1 = no_inter_sub[0][0];
      double y1 = no_inter_sub[0][1];
      double x2 = no_inter_sub.back()[0];
      double y2 = no_inter_sub.back()[1];
      double xd = start_direction[0];
      double yd = start_direction[1];
      yd = -yd;
      
      double d1 = xd / std::hypot(xd, yd);
      double d2 = xd / std::hypot(xd, yd);
      double d3 = (x1 - x0) / std::hypot(x1 - x0, y1 - y0);
      double d4 = (y1 - y0) / std::hypot(x1 - x0, y1 - y0);
      double d5 = (x2 - x0) / std::hypot(x2 - x0, y2 - y0);
      double d6 = (y2 - y0) / std::hypot(x2 - x0, y2 - y0);

      // Add points only in front ot the vehicle
      double dot1 = (d1 * d3) + (d2 * d4);
      double dot2 = (d1 * d5) + (d2 * d6);
      double angle1 = (acos(std::min(std::max(dot1, -1.0), 1.0))) * (180.0 / M_PI);
      double angle2 = (acos(std::min(std::max(dot2, -1.0), 1.0))) * (180.0 / M_PI);

      if(angle1 <= 110)
      {
        LineSegment seg;
        seg.point = {x1, y1};
        seg.value = i;
        perimeter_points.push_back(seg);
      }

      if(angle2 <= 110)
      {
        LineSegment seg;
        seg.point = {x2, y2};
        seg.value = i;
        perimeter_points.push_back(seg);
      }
    }
    if(int(perimeter_points.size()) == 0)
    {
      LineSegment seg;
      seg.point = no_inter_sections[0].back();
      seg.value = 0;
      perimeter_points.push_back(seg);
    }

    // Find biggest traingle
    x0 = start_point[0];
    y0 = start_point[1];
    std::vector<LineSegment> biggest_perimeter_points;
    float biggest_perimeter = 0.0;
    if(int(perimeter_points.size()) > 1)
    {
      for(LineSegment point1 : perimeter_points)
      {
        for(LineSegment point2 : perimeter_points)
        {
          if(point1.point[0] == point2.point[0] && point1.point[1] == point2.point[1])
          {
            continue;
          }

          double x1 = point1.point[0];
          double y1 = point1.point[1];
          x1 = x1 - x0;
          y1 = y1 - y0;
          double x2 = point2.point[0];
          double y2 = point2.point[1];
          x2 = x2 - x0;
          y2 = y2 - y0;

          double perimeter = std::hypot(x1, y1) + std::hypot(x2, y2) + std::hypot(x1 - x2, y1 - y2);

          if(perimeter > biggest_perimeter)
          {
            biggest_perimeter = perimeter;
            biggest_perimeter_points = {point1, point2};
          }
        }
      }
    }
    else
    {
      biggest_perimeter_points = perimeter_points;
    }

    // Choose closest path as first path
    x0 = start_point[0];
    y0 = start_point[1];
    std::vector<std::vector<double>> turn_right_sections = no_inter_sections[0];
    for(std::vector<std::vector<double>> section : no_inter_sections)
    {
      double smallest_dist = 1e9;
      for(std::vector<double> point : section)
      {
        double x1 = point[0];
        double y1 = point[1];
        double dist = std::hypot(x1 - x0, y1 - y0);

        if(dist < smallest_dist)
        {
          smallest_dist = dist;
        }
      }
      if(smallest_dist < 1.0)
      {
        turn_right_sections = section;
        break;
      }
    }

    // Turn right path
    if(biggest_perimeter_points[0].value != 0)
    {
      std::vector<std::vector<double>> section_to_add = no_inter_sections[int(biggest_perimeter_points[0].value)];
        
      if(biggest_perimeter_points[0].point[0] == section_to_add[0][0] && biggest_perimeter_points[0].point[1] == section_to_add[0][1])
      {
        std::reverse(section_to_add.begin(), section_to_add.end());
      }

      double smallest_dist = 1e9;
      int smallest_idx1 = int(turn_right_sections.size());
      int smallest_idx2 = int(section_to_add.size());

      for(int i=0; i<int(turn_right_sections.size()); i++)
      {
        std::vector<double> point1 = turn_right_sections[i];
        double xp1 = point1[0];
        double yp1 = point1[1];

        for(int j=0; j<int(section_to_add.size()); j++)
        {
          std::vector<double> point2 = section_to_add[j];
          double xp2 = point2[0];
          double yp2 = point2[1];

          double dist = std::hypot(xp1 - xp2, yp1 - yp2);

          if(dist < smallest_dist)
          {
            smallest_dist = dist;

            if(smallest_idx2 == 0)
            {
              smallest_idx2 = 1;
            }
            else
            {
              smallest_idx1 = i;
              smallest_idx2 = j;
            }
          }
        }
      }
      // turn_right_sections = std::vector<std::vector<double>>(turn_right_sections.begin(), turn_right_sections.end() - (int(turn_right_sections.size()) - smallest_idx1));
      // section_to_add = std::vector<std::vector<double>>(section_to_add.begin() + smallest_idx2, section_to_add.end());
      turn_right_sections.erase(turn_right_sections.begin() + smallest_idx1, turn_right_sections.end());
      turn_right_sections.insert(turn_right_sections.end(), section_to_add.begin() + smallest_idx2, section_to_add.end());
      // for(std::vector<double> point : section_to_add)
      // {
      //   turn_right_sections.push_back(point);
      // }
    }

    // Smooth trajectory with Bezier curve
    std::vector<std::vector<double>> path_right = LidarTrajectory::bezier_curve(turn_right_sections, 15);
    std::reverse(path_right.begin(), path_right.end());

    // Reverse all sections
    std::vector<std::vector<double>> section_turn_left;
    if(int(biggest_perimeter_points.size()) > 0)
    {
      if(int(no_inter_sections.size()) > biggest_perimeter_points.back().value)
      {
        section_turn_left = no_inter_sections[biggest_perimeter_points.back().value];
      }
    }
    for(std::vector<std::vector<double>> &section : no_inter_sections)
    {
      std::reverse(section.begin(), section.end());
    }
    std::reverse(no_inter_sections.begin(), no_inter_sections.end());

    // Choose closest path as first path
    x0 = start_point[0];
    y0 = start_point[1];
    std::vector<std::vector<double>> turn_left_sections = no_inter_sections.back();
    for(std::vector<std::vector<double>> section : no_inter_sections)
    {
      double smallest_dist = 1e9;
      for(std::vector<double> point : section)
      {
        double x1 = point[0];
        double y1 = point[1];
        double dist = std::hypot(x1 - x0, y1 - y0);

        if(dist < smallest_dist)
        {
          smallest_dist = dist;
        }
      }
      if(smallest_dist < 1.0)
      {
        turn_left_sections = section;
        break;
      }
    }

    // Turn left path
    if(biggest_perimeter_points.back().value != int(no_inter_sections.size()) - 1)
    {
      std::vector<std::vector<double>> section_to_add = section_turn_left;

      if(biggest_perimeter_points.back().point[0] == section_to_add[0][0] && biggest_perimeter_points.back().point[1] == section_to_add[0][1])
      {
        std::reverse(section_to_add.begin(), section_to_add.end());
      }

      double smallest_dist = 1e9;
      double smallest_idx1 = int(turn_left_sections.size());
      double smallest_idx2 = int(section_to_add.size());

      for(int i=0; i<int(turn_left_sections.size()); i++)
      {
        std::vector<double> point1 = turn_left_sections[i];
        double xp1 = point1[0];
        double yp1 = point1[1];

        for(int j=0; j<int(section_to_add.size()); j++)
        {
          std::vector<double> point2 = section_to_add[j];
          double xp2 = point2[0];
          double yp2 = point2[1];

          double dist = std::hypot(xp1 - xp2, yp1 - yp2);

          if(dist < smallest_dist)
          {
            smallest_dist = dist;

            if(smallest_idx2 == 0)
            {
              smallest_idx2 = 1;
            }
            else
            {
              smallest_idx1 = i;
              smallest_idx2 = j;
            }
          }
        }
      }
      turn_left_sections = std::vector<std::vector<double>>(turn_left_sections.begin(), turn_left_sections.end() - (int(turn_left_sections.size()) - smallest_idx1));
      section_to_add = std::vector<std::vector<double>>(section_to_add.begin() + smallest_idx2, section_to_add.end());
      for(std::vector<double> point : section_to_add)
      {
        turn_left_sections.push_back(point);
      }
    }
    
    std::vector<std::vector<double>> path_left = LidarTrajectory::bezier_curve(turn_left_sections, 15);
    std::reverse(path_left.begin(), path_left.end());
        
    // // Print out the vector
    // std::cout << "Input" << std::endl;
    // std::cout << "{";
    // for(auto n : turn_left_sections)
    //   std::cout << "{" << n[0] << ", " << n[1] << "},";
    // std::cout << "}";
    // std::cout << std::endl;

    // // Print out the vector
    // std::cout << "Output" << std::endl;
    // std::cout << "{";
    // for(auto n : path_left)
    //   std::cout << "{" << n[0] << ", " << n[1] << "},";
    // std::cout << "}";
    // std::cout << std::endl;

    // Populate the message
    trajectory.header = laser.header;
    trajectory.header.frame_id = "map";

    for (auto & point : path_right) {
      TrajectoryPoint trajectory_point;
      Pose pose;
      auto q = tier4_autoware_utils::createQuaternionFromYaw(start_angle + (M_PI/2)); //TODO Calculate the angle
      pose.position.x = point[0];
      pose.position.y = -point[1];
      pose.position.z = 0.0;
      pose.orientation = q;

      trajectory_point.pose = pose;

      trajectory_point.longitudinal_velocity_mps = 1.0; // 5.167099952697754; //TODO Better velocity estiomatoin

      trajectory.points.push_back(trajectory_point);
    }
  }
  return trajectory;
}

unsigned long long LidarTrajectory::comb(const unsigned n, const unsigned i)
{
  if (n == i || i == 0) 
  {
    return 1;
  }
  return comb(n - 1, i - 1) * n / i;
}

std::vector<double> LidarTrajectory::bernstein_poly(int i, int n, const std::vector<double>& t)
{
  unsigned long long term1 = comb(n, i);
  std::vector<double> term2;
  for(double val : t)
  {
    term2.push_back(std::pow(val, n - i));
  }
  std::vector<double> term3;
  for(double val : t)
  {
    term3.push_back(std::pow(1 - val, i));
  }
  std::vector<double> result;
  for(int i = 0; i < int(term2.size()); i++)
  {
    result.push_back(term1 * term2[i] * term3[i]);
  }
  return result;
}

std::vector<std::vector<double>> LidarTrajectory::bezier_curve(const std::vector<std::vector<double>>& points, const int nTimes)
{
  int nPoints = int(points.size());

  std::vector<double> xPoints;
  std::vector<double> yPoints;
  for(const std::vector<double>& point : points)
  {
    xPoints.push_back(point[0]);
    yPoints.push_back(point[1]);
  }

  std::vector<double> t;
  double delta = 1.0 / (nTimes - 1);
  for(int i = 0; i < nTimes - 1; ++i)
  {
    t.push_back(delta * i);
  }
  t.push_back(1.0);

  std::vector<std::vector<double>> polynomial_array;
  for(int i = 0; i < nPoints; i++)
  { 
    polynomial_array.push_back(bernstein_poly(i, nPoints - 1, t));
  }
  std::vector<std::vector<double>> result;
  for (int i = 0; i < nTimes; i++)
  {
    double xsum = 0;
    double ysum = 0;
    for (int j = 0; j < nPoints; j++)
    {
      xsum += xPoints[j] * polynomial_array[j][i];
      ysum += yPoints[j] * polynomial_array[j][i];
    }
    std::vector<double> point = {xsum, ysum};
    result.push_back(point);
  }
  return result;
}

}  // namespace lidar_trajectory
