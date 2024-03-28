cmake_minimum_required(VERSION 3.14)
project(lidar_trajectory)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  include/lidar_trajectory/lidar_trajectory.hpp
  include/lidar_trajectory/visibility_control.hpp
  src/lidar_trajectory.cpp
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  include/lidar_trajectory/lidar_trajectory_node.hpp
  src/lidar_trajectory_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "lidar_trajectory::LidarTrajectoryNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_ros_isolated_gtest(test_lidar_trajectory
    test/test_lidar_trajectory.cpp
  )
  ament_target_dependencies(test_lidar_trajectory ${TEST_SOURCES})
  target_link_libraries(test_lidar_trajectory ${PROJECT_NAME})
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
