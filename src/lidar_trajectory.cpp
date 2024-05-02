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

  // std::cout << "Calcualting trajectory" << std::endl;
  // if(this->laser_scan != nullptr && this->odometry != nullptr)
  // {
  //   std::cout << "Calcualted trajectory" << std::endl;
  // }


  // This values are temporary and for debug only
  // They shoud be replaced with values from simulation
  double start_angle = -M_PI / 2; //From odometry->pose.orientation <--- This is quaternion it neads to be recalculated
  std::vector<double> start_direction = {sin(-start_angle), cos(start_angle)};

  // From sizeof(laser_scan->ranges)
  double distances[] = {1.299287723640519, 1.3107345212238442, 1.2649528675627137, 1.2655797542952876, 1.2655797542952876, 1.2207040246584364, 1.2099352957584153, 1.2116821803291924, 1.2116821803291924, 1.213862250786279, 1.1580158578708848, 1.1609801644145588, 1.1609801644145588, 1.164391140810937, 1.1096295231429474, 1.1004178416708053, 1.1139101501433464, 1.1049734639370412, 1.1099868725773017, 1.065257844172956, 1.056910569105691, 1.0487489753884291, 1.0628973655547458, 1.055032728293225, 1.0617774125082606, 1.0544060381706326, 1.0108800677963936, 1.0108800677963936, 1.0039251408411793, 1.012056345989947, 1.0056355184433319, 1.0056355184433319, 0.9994380071954023, 1.0086544662940373, 1.0030029602140578, 0.9975844975725507, 0.9615518336242288, 0.9564514582018043, 0.9516016187576931, 0.9470061644592543, 0.9626510644302885, 0.9583845221652928, 0.9543759733872643, 0.9543759733872643, 0.9506286817719328, 0.9471457482413463, 0.9439300991024385, 0.9570041606219292, 0.9570041606219292, 0.9543759733872643, 0.952018287369395, 0.9499331166815305, 0.9499331166815305, 0.9481222593244392, 0.9465872896148217, 0.9453295514633258, 0.9453295514633258, 0.9443501525679092, 0.9436499595802867, 0.9432295942941903, 0.9105691056910569, 0.943089430894309, 0.9432295942941903, 0.9436499595802867, 0.9443501525679092, 0.9443501525679092, 0.9453295514633258, 0.9465872896148217, 0.9481222593244392, 0.9481222593244392, 0.9499331166815305, 0.952018287369395, 0.9543759733872643, 0.9543759733872643, 0.9409844745515634, 0.9439300991024385, 0.9471457482413463, 0.9982468603688344, 1.0018160520812593, 1.0056355184433319, 1.0097024192689446, 0.9940001115823041, 0.9983792801956911, 1.0030029602140575, 1.0438213883384737, 0.9975844975725507, 0.9975844975725507, 1.0030029602140578, 0.9934679915700897, 0.9994380071954023, 0.9994380071954023, 1.0056355184433319, 0.9971868687129732, 1.0039251408411793, 1.0039251408411793, 1.0472352642669671, 1.0544060381706326, 1.0473614904187263, 1.055032728293225, 1.0407774312718867, 1.0487489753884291, 1.056910569105691, 1.1014985076058503, 1.0962051245972797, 1.1049734639370412, 1.1004178416708053, 1.1004178416708053, 1.190883474705727, 1.2002819859486553, 1.1967523859939642, 1.2065435095342534, 1.2392974967343888, 1.2493900951088486, 1.2470601293537569, 1.2575109789258736, 1.3017273209512477, 1.3125486875193864, 1.3005080943450282, 1.2537207496716252, 1.299287723640519, 1.287739178746233, 1.299287723640519, 1.34566968040329, 1.3575044855116618, 1.358477956045881, 1.4057284388000642, 1.4074201631468228, 1.4552618429440292, 1.45762177762699, 1.5516319269053391, 1.6001189723762659, 1.603255330724357, 1.6652978713919884, 1.6558243405659794, 1.695818811353748, 1.7001786502187917, 1.704837533587782, 1.7097930169324227, 1.7512731080112207, 1.8153945430174045, 1.8071471856194867, 1.958387916343718, 2.01534204115944, 2.058372179052384, 2.116133819303265, 2.1161962891203765, 2.109125491231357, 2.16797425422382, 2.3658396892834874, 2.470314199456141, 2.515337314517364, 2.6209142715368343, 2.7177342979871453, 2.803404821067751, 2.8646050442636235, 2.9720979765889806, 3.110964747319611, 3.2815731473440533, 3.4217888651420165, 3.5781595259618517, 3.735062345720545, 3.87667647993842, 4.129249003903937, 4.232798606582791, 4.636912596137476, 4.93958476529284, 4.994279231080945, 4.998856369971165, 5.004327560760025, 4.994596855869396, 5.00181737082017, 4.9937762838494955, 5.002742324177061, 4.996396348272721, 5.007100517126121, 5.002451642979306, 4.998644802793918, 4.995681921525983, 4.993564501439331, 4.992293618310038, 5.008130081300813, 4.992293618310038, 4.993564501439331, 4.995681921525983, 4.998644802793918, 5.002451642979306, 5.007100517126121, 4.996396348272721, 5.002742324177061, 4.9937762838494955, 5.00181737082017, 4.994596855869396, 5.004327560760025, 4.998856369971165, 4.994279231080945, 4.942768496213944, 4.640303996277648, 4.3387525411792955, 4.133056990883816, 3.8294322717034723, 3.739271784729132, 3.3815274663702857, 3.323602691898121, 3.286363503178168, 3.116017396087079, 2.8692622294400976, 2.7311749384440676, 2.721477166323225, 2.7177342979871453, 2.472934996221771, 2.4190510094602278, 2.425436404906715, 2.3724797552436385, 2.3133474539276833, 2.269210409680587, 2.1669984035001106, 2.1235548250165213, 2.066000673075589, 2.023132792605721, 1.9664043288533049, 1.9242032120308312, 1.9185613840551468, 1.8631904404073127, 1.8132815426927675, 1.7723577235772359, 1.767876785031695, 1.6104950262652857, 1.6199072231846565, 1.5577541283899172, 1.567483048270836, 1.505975495091816, 1.5160367323774322, 1.4678343377021144, 1.4656712501886136, 1.4638662356488816, 1.4624206199543022, 1.473317891391778, 1.4146341463414633, 1.4146341463414633, 1.4142603001017582, 1.3682711861080603, 1.35692006796911, 1.3575044855116618, 1.3125486875193864, 1.324679431612212, 1.3139579740381737, 1.269959296895391, 1.2596117337797927, 1.2621280240861807, 1.2521381013449402, 1.1609801644145588, 1.164391140810937, 1.164391140810937, 1.168244874581207, 1.2176679822340848, 1.1635961406708841, 1.15481501818307, 1.1685842997613625, 1.160068876860941, 1.1657527368412957, 1.1213619284271004, 1.1134353376110564, 1.1056910569105691, 1.1126039281012032, 1.1126039281012032, 1.1199463647299228, 1.1129603225136653, 1.0698396194859174, 1.0632704210908497, 1.0719381934211063, 1.0719381934211063, 1.0658781539234072, 1.0600329114150648, 1.0697160457494976, 1.0643888031904483, 1.0643888031904483, 1.0592843886115337, 1.0185664886742, 1.014013777409357, 1.0097024192689446, 1.0097024192689446, 1.021417806474108, 1.0176575758256163, 1.0141441385048895, 1.0108800677963936, 1.026839245778663, 1.0238739181418643, 1.0211589247549646, 1.0186962671659892, 1.0164877793878233, 1.0164877793878233, 1.0145351212882274, 1.0128397725136544, 1.0114030269973324, 1.0264529497292116, 1.0264529497292116, 1.0255510264209091, 1.0249063095492308, 1.0245192846844464, 1.024390243902439, 0.975609756097561, 1.0245192846844464, 1.0249063095492308, 1.0255510264209091, 1.0264529497292116, 1.0276114022325178, 1.0276114022325178, 1.0128397725136544, 1.0145351212882274, 1.0164877793878233, 1.0186962671659892, 1.0186962671659892, 1.0211589247549646, 1.0238739181418643, 1.026839245778663, 1.0300527456723156, 1.0141441385048895, 1.0176575758256163, 1.021417806474108, 1.0254221154749632, 1.014013777409357, 1.014013777409357, 1.0185664886742, 0.9719443339107939, 0.9769638299707188, 1.0129702846941477, 1.0183068820716212, 1.0238739181418643, 1.0658781539234072, 1.0719381934211063, 1.0782094081366116, 1.0782094081366116, 1.0698396194859174, 1.0766143321786568, 1.1199463647299228, 1.1271236959755038, 1.120064396702083, 1.120064396702083, 1.1134353376110564, 1.1213619284271004, 1.1294669910121793, 1.1740017564998178, 1.1685842997613625, 1.1772627225949888, 1.1635961406708841, 1.1725370001508908, 1.2268613248544797, 1.2135354898891246, 1.2229761103397572, 1.2195121951219512, 1.2293371168413527, 1.2265380270338935, 1.2241645674613797, 1.2241645674613797, 1.222219217753895, 1.2787767636443421, 1.3355127933906386, 1.3234813492763988, 1.368657594245136, 1.3682711861080603};
  int num_points = sizeof(distances) / sizeof(*distances);
  double scan_angle = 0.01308996938995747; // From laser_scan->angle_increment
  double offset_angle = (((M_PI * 2) / scan_angle) - num_points) / 2;
  double max_dist = 5.0; // From laser_scan->range_max
  std::vector<double> start_point = {12.195121951219512, 12.439024390243903};



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

  class LineSegment
  {
    public:
      std::vector<double> point;
      double value;
  };

  // Create point sections 
  std::vector<std::vector<LineSegment>> line_sections;
  std::vector<LineSegment> line_subsection;
  float threshold = 0.3;
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

  // Print out the vector
  for(auto n : turn_right_sections)
    std::cout << int(turn_right_sections.size()) << " " << n[0] << " " << n[1] << " | ";
  std::cout << std::endl;

  return trajectory;
}
}  // namespace lidar_trajectory
