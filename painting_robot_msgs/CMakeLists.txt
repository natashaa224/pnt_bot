cmake_minimum_required(VERSION 3.8)
project(painting_robot_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
 "msg/BmsStatus.msg"
  "msg/WallPose.msg"
  "msg/WallStatus.msg"
  "msg/EnvStatus.msg"
  "msg/CliffStatus.msg"
  "msg/UltrasoundStatus.msg"
  "msg/CollisionWarning.msg"
  "msg/CornerParams.msg"
  "msg/FixturePose.msg"
  "msg/WallDepthProfile.msg"
  "msg/ZAxisCmd.msg"
  "msg/WristCmd.msg"
  "msg/SprayCmd.msg"
  "msg/PaintMotionModes.msg"
  "msg/SafetyMode.msg"
  "msg/SafetyCommand.msg"
  "msg/McuStatus.msg"
  "msg/DiStatus.msg"
  "msg/EncoderFeedback.msg"
  "msg/PaintHeadStatus.msg"
  "msg/StripMap.msg"
  "msg/RobotCommand.msg"
  "msg/GantryLimitCmd.msg"
  "msg/MotionStatus.msg"
  "msg/JobStats.msg"
  DEPENDENCIES std_msgs
  geometry_msgs
)
ament_package()
