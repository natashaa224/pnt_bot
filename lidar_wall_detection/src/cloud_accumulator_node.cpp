#include "lidar_wall_detection/cloud_accumulator.hpp"
#include <algorithm>
#include <pcl/filters/voxel_grid.h>
#include <cmath>
#include <limits>
#include <vector>
#include <sensor_msgs/point_cloud2_iterator.hpp>

static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
    return std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

CloudAccumulator::CloudAccumulator()
: rclcpp::Node("cloud_accumulator"),
  has_last_pose_(false)
{
    fixed_frame_ = this->declare_parameter<std::string>("fixed_frame", "odom");
    lidar_frame_ = this->declare_parameter<std::string>("lidar_frame", "base_scan");
    input_cloud_topic_ = this->declare_parameter<std::string>("input_cloud_topic", "/wall_scan_cloud");
    max_cloud_size_ = this->declare_parameter<int>("max_cloud_size", 500000);
    accumulation_distance_ = this->declare_parameter<double>("accumulation_distance", 0.02);
    voxel_size_ = this->declare_parameter<double>("voxel_size", 0.03);
    window_forward_ = this->declare_parameter<double>("window_forward", 2.5);
    window_side_ = this->declare_parameter<double>("window_side", 1.0);
    min_height_ = this->declare_parameter<double>("min_height", 0.0);
    max_height_ = this->declare_parameter<double>("max_height", 1.5);
    cutout_gap_threshold_ = this->declare_parameter<double>("cutout_gap_threshold", 0.08);
    min_cutout_points_ = this->declare_parameter<int>("min_cutout_points", 8);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_cloud_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&CloudAccumulator::cloudCallback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/cloud_3d",
        rclcpp::SensorDataQoS());

    edge_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/cutout_edges",
        rclcpp::SensorDataQoS());

    RCLCPP_INFO(this->get_logger(),
                "Cloud Accumulator started. Fixed frame: %s, Lidar frame: %s",
                fixed_frame_.c_str(), lidar_frame_.c_str());

    RCLCPP_INFO(this->get_logger(),
                "Input cloud topic: %s",
                input_cloud_topic_.c_str());

    if (fixed_frame_ == lidar_frame_) {
        RCLCPP_WARN(this->get_logger(),
                    "fixed_frame and lidar_frame are both '%s'. The accumulated cloud will stay in the moving lidar frame, so past data will not appear fixed in space.",
                    fixed_frame_.c_str());
    }
}

double CloudAccumulator::computeDistance(
    const geometry_msgs::msg::PoseStamped & p1,
    const geometry_msgs::msg::PoseStamped & p2)
{
    double dx = p1.pose.position.x - p2.pose.position.x;
    double dy = p1.pose.position.y - p2.pose.position.y;
    double dz = p1.pose.position.z - p2.pose.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void CloudAccumulator::downSample(
    pcl::PointCloud<pcl::PointXYZ> & cloud,
    double voxel_size)
{
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud.makeShared());
    voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel_filter.filter(cloud);
}

// Keep only points inside a forward-facing window in the robot frame.
void CloudAccumulator::pruneCloud(
    pcl::PointCloud<pcl::PointXYZ> & cloud,
    const geometry_msgs::msg::PoseStamped & center,
    double forward_distance,
    double side_distance)
{
    if (forward_distance <= 0.0 || side_distance <= 0.0) {
        return;
    }

    const double yaw = yawFromQuaternion(center.pose.orientation);
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);

    pcl::PointCloud<pcl::PointXYZ> kept;
    kept.reserve(cloud.size());

    for (const auto & point : cloud.points) {
        const double dx = point.x - center.pose.position.x;
        const double dy = point.y - center.pose.position.y;
        const double robot_x =  cos_yaw * dx + sin_yaw * dy;
        const double robot_y = -sin_yaw * dx + cos_yaw * dy;

        if (robot_x >= -0.20 && robot_x <= forward_distance &&
            std::abs(robot_y) <= side_distance) {
            kept.points.push_back(point);
        }
    }

    kept.width = static_cast<uint32_t>(kept.points.size());
    kept.height = 1;
    kept.is_dense = cloud.is_dense;
    cloud.swap(kept);
}

// Keep only the vertical slice where the cutout edges are expected.
void CloudAccumulator::filterByHeight(
    pcl::PointCloud<pcl::PointXYZ> & cloud,
    double min_height,
    double max_height)
{
    if (max_height <= min_height) {
        return;
    }

    pcl::PointCloud<pcl::PointXYZ> kept;
    kept.reserve(cloud.size());

    for (const auto & point : cloud.points) {
        if (point.z >= min_height && point.z <= max_height) {
            kept.points.push_back(point);
        }
    }

    kept.width = static_cast<uint32_t>(kept.points.size());
    kept.height = 1;
    kept.is_dense = cloud.is_dense;
    cloud.swap(kept);
}

void CloudAccumulator::publishEdgeValues(const pcl::PointCloud<pcl::PointXYZ> & cloud)
{
    std_msgs::msg::Float32MultiArray msg;
    msg.data.reserve(1 + cloud.size() * 2);

    if (cloud.empty()) {
        msg.data = {0.0F};
        edge_pub_->publish(msg);
        return;
    }

    auto points = cloud.points;
    std::sort(points.begin(), points.end(), [](const pcl::PointXYZ & a, const pcl::PointXYZ & b) {
        return a.z < b.z;
    });

    struct GapRange
    {
        float bottom_z;
        float top_z;
    };

    std::vector<GapRange> gap_ranges;

    for (std::size_t i = 0; i + 1 < points.size(); ++i) {
        const float gap = points[i + 1].z - points[i].z;
        if (gap >= cutout_gap_threshold_) {
            const std::size_t lower_count = i + 1;
            const std::size_t upper_count = points.size() - (i + 1);

            if (lower_count < static_cast<std::size_t>(min_cutout_points_) ||
                upper_count < static_cast<std::size_t>(min_cutout_points_)) {
                continue;
            }

            gap_ranges.push_back(GapRange{points[i].z, points[i + 1].z});
        }
    }

    msg.data.push_back(gap_ranges.empty() ? 0.0F : 1.0F);
    msg.data.push_back(static_cast<float>(gap_ranges.size()));

    for (const auto & range : gap_ranges) {
        msg.data.push_back(range.bottom_z);
        msg.data.push_back(range.top_z);
    }

    edge_pub_->publish(msg);
}

void CloudAccumulator::cloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
    if (!cloud_msg) {
        RCLCPP_WARN(this->get_logger(), "Received null PointCloud2 message");
        return;
    }

    try {
        geometry_msgs::msg::TransformStamped transform =
            tf_buffer_->lookupTransform(
                fixed_frame_,
                lidar_frame_,
                cloud_msg->header.stamp,
                std::chrono::milliseconds(100));

        geometry_msgs::msg::PoseStamped current_pose;
        current_pose.header.frame_id = fixed_frame_;
        current_pose.header.stamp = cloud_msg->header.stamp;
        current_pose.pose.position.x = transform.transform.translation.x;
        current_pose.pose.position.y = transform.transform.translation.y;
        current_pose.pose.position.z = transform.transform.translation.z;
        current_pose.pose.orientation = transform.transform.rotation;

        bool should_accumulate = !has_last_pose_ || 
                                 computeDistance(current_pose, last_pose_) >= accumulation_distance_;

        if (!should_accumulate) {
            return;
        }

        sensor_msgs::msg::PointCloud2 transformed_cloud;
        tf2::doTransform(*cloud_msg, transformed_cloud, transform);

        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl_cloud.reserve(transformed_cloud.width * transformed_cloud.height);

        sensor_msgs::PointCloud2ConstIterator<float> x_iter(transformed_cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> y_iter(transformed_cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> z_iter(transformed_cloud, "z");

        for (; x_iter != x_iter.end(); ++x_iter, ++y_iter, ++z_iter) {
            pcl_cloud.push_back(pcl::PointXYZ(*x_iter, *y_iter, *z_iter));
        }

        accumulated_cloud_ += pcl_cloud;

        last_pose_ = current_pose;
        has_last_pose_ = true;

        pcl::PointCloud<pcl::PointXYZ> detection_cloud = accumulated_cloud_;

        if (detection_cloud.size() > static_cast<std::size_t>(max_cloud_size_)) {
            RCLCPP_INFO(this->get_logger(),
                        "Detection cloud size %zu exceeded max %d, downsampling with voxel size %f",
                        detection_cloud.size(), max_cloud_size_, voxel_size_);
            downSample(detection_cloud, voxel_size_ * 2.0);
        }

        filterByHeight(detection_cloud, min_height_, max_height_);
        pruneCloud(detection_cloud, current_pose, window_forward_, window_side_);

        publishEdgeValues(detection_cloud);

        sensor_msgs::msg::PointCloud2 output_msg;
        sensor_msgs::PointCloud2Modifier modifier(output_msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(accumulated_cloud_.size());

        sensor_msgs::PointCloud2Iterator<float> out_x(output_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> out_y(output_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> out_z(output_msg, "z");

        for (const auto & point : accumulated_cloud_) {
            *out_x = point.x;
            *out_y = point.y;
            *out_z = point.z;
            ++out_x;
            ++out_y;
            ++out_z;
        }
        output_msg.header.frame_id = fixed_frame_;
        output_msg.header.stamp = cloud_msg->header.stamp;

        publisher_->publish(output_msg);

        RCLCPP_DEBUG(this->get_logger(),
                     "Accumulated cloud size: %zu points",
                     accumulated_cloud_.size());

    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(),
                    "Transform lookup failed: %s", ex.what());
        return;
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CloudAccumulator>());
    rclcpp::shutdown();
    return 0;
}