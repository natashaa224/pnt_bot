#ifndef LIDAR_WALL_DETECTION__CLOUD_ACCUMULATOR_HPP_
#define LIDAR_WALL_DETECTION__CLOUD_ACCUMULATOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class CloudAccumulator : public rclcpp::Node
{
public:
    CloudAccumulator();

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr edge_pub_;

    std::string fixed_frame_;
    std::string lidar_frame_;
    std::string input_cloud_topic_;

    pcl::PointCloud<pcl::PointXYZ> accumulated_cloud_;

    int max_cloud_size_;
    double accumulation_distance_;
    double voxel_size_;
    double window_forward_;
    double window_side_;
    double min_height_;
    double max_height_;
    double cutout_gap_threshold_;
    int min_cutout_points_;

    geometry_msgs::msg::PoseStamped last_pose_;
    bool has_last_pose_;

    double computeDistance(
        const geometry_msgs::msg::PoseStamped & p1,
        const geometry_msgs::msg::PoseStamped & p2);

    void downSample(
        pcl::PointCloud<pcl::PointXYZ> & cloud,
        double voxel_size);

    void pruneCloud(
        pcl::PointCloud<pcl::PointXYZ> & cloud,
        const geometry_msgs::msg::PoseStamped & center,
        double forward_distance,
        double side_distance);

    void filterByHeight(
        pcl::PointCloud<pcl::PointXYZ> & cloud,
        double min_height,
        double max_height);

    void publishEdgeValues(const pcl::PointCloud<pcl::PointXYZ> & cloud);
};

#endif  // WALL_3D_DETECTION__CLOUD_ACCUMULATOR_HPP_