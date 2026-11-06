cmake_minimum_required(VERSION 3.8)
project(painting_robot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(painting_robot_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# ==============================================================================
# C++ Nodes
# ==============================================================================
add_executable(wall_detection_node src/wall_detection_node.cpp)
target_include_directories(wall_detection_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(wall_detection_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  painting_robot_msgs
)

install(TARGETS
  wall_detection_node
  DESTINATION lib/${PROJECT_NAME}
)

# ==============================================================================
# Python Modules & Scripts
# ==============================================================================
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/wall_motion_node
  scripts/wall_detection_node_py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
