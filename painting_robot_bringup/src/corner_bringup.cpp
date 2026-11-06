// Copyright 2026 VRai / Antigravity
// C++ Bringup Executable demonstrating programmatic C++ instantiation of WallDetectionNode

#include <memory>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "lidar_wall_detection/wall_detection_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    // Allow parameter overrides from command-line arguments (--ros-args --params-file ...)
    options.automatically_declare_parameters_from_overrides(true);

    RCLCPP_INFO(rclcpp::get_logger("corner_bringup"), "Launching C++ corner_bringup with lidar_wall_detection...");

    // Directly instantiate the WallDetectionNode from the separate lidar_wall_detection package
    auto wall_detection_node = std::make_shared<lidar_wall_detection::WallDetectionNode>(options);

    // Multi-threaded executor allowing parallel callback processing
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(wall_detection_node);

    RCLCPP_INFO(rclcpp::get_logger("corner_bringup"), "corner_bringup C++ executor spinning. Press Ctrl+C to stop.");
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
