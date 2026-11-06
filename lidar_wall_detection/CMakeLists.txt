cmake_minimum_required(VERSION 3.8)
project(lidar_wall_detection)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(painting_robot_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# ── Library: lidar_wall_detection_lib ──────────────────────────────────────────
add_library(lidar_wall_detection_lib SHARED
  src/wall_detection_node.cpp
)

target_include_directories(lidar_wall_detection_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(lidar_wall_detection_lib
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  painting_robot_msgs
)

# ── Executable: wall_detection_node (Horizontal RANSAC/PCA) ─────────────────────
add_executable(wall_detection_node
  src/wall_detection_main.cpp
)

target_link_libraries(wall_detection_node
  lidar_wall_detection_lib
)

ament_target_dependencies(wall_detection_node
  rclcpp
)

# ── Executable: laser_scan_box_filter (Vertical scan box filter) ───────────────
add_executable(laser_scan_box_filter
  src/laser_scan_box_filter.cpp
)

ament_target_dependencies(laser_scan_box_filter
  rclcpp
  sensor_msgs
  visualization_msgs
)

# ── Executable: obstacle_detector (Vertical wall protrusions) ───────────────────
add_executable(obstacle_detector
  src/obstacle_detector.cpp
)

ament_target_dependencies(obstacle_detector
  rclcpp
  sensor_msgs
  visualization_msgs
  geometry_msgs
)

# ── Executable: cutout_window_detector (Wall voids/openings/windows) ───────────
add_executable(cutout_window_detector
  src/cutout_window_detector.cpp
)

ament_target_dependencies(cutout_window_detector
  rclcpp
  sensor_msgs
  visualization_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ── Executable: wall_scan_to_pointcloud_node (Vertical scan -> PointCloud2) ────
add_executable(wall_scan_to_pointcloud_node
  src/wall_scan_to_pointcloud_node.cpp
)

ament_target_dependencies(wall_scan_to_pointcloud_node
  rclcpp
  sensor_msgs
)

# ── Executable: cloud_accumulator_node (3D Wall PointCloud Accumulator) ─────────
add_executable(cloud_accumulator_node
  src/cloud_accumulator_node.cpp
)

ament_target_dependencies(cloud_accumulator_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
  pcl_conversions
)

target_link_libraries(cloud_accumulator_node
  ${PCL_LIBRARIES}
)

# ── Installation ───────────────────────────────────────────────────────────────
install(
  TARGETS lidar_wall_detection_lib
  EXPORT export_lidar_wall_detection
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS
    wall_detection_node
    laser_scan_box_filter
    obstacle_detector
    cutout_window_detector
    wall_scan_to_pointcloud_node
    cloud_accumulator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# ── Exports ────────────────────────────────────────────────────────────────────
ament_export_include_directories(include)
ament_export_libraries(lidar_wall_detection_lib)
ament_export_targets(export_lidar_wall_detection)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  painting_robot_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
  pcl_conversions
)

ament_package()
