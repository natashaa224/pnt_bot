#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class WallScanToPointCloud : public rclcpp::Node
{
public:
  WallScanToPointCloud()
  : Node("wall_scan_to_pointcloud")
  {
    scan_topic_ = this->declare_parameter<std::string>("scan_topic", "/wall_scan");
    cloud_topic_ = this->declare_parameter<std::string>("cloud_topic", "/wall_scan_cloud");
    target_frame_ = this->declare_parameter<std::string>("target_frame", "laser");
    publish_z_ = this->declare_parameter<double>("publish_z", 0.0);

    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&WallScanToPointCloud::scanCallback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      cloud_topic_,
      rclcpp::SensorDataQoS());

    RCLCPP_INFO(this->get_logger(),
      "Converting %s to %s in frame %s",
      scan_topic_.c_str(), cloud_topic_.c_str(), target_frame_.c_str());
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (!msg) {
      RCLCPP_WARN(this->get_logger(), "Received null LaserScan message");
      return;
    }

    std::size_t point_count = 0;
    for (const float range : msg->ranges) {
      if (std::isfinite(range) && range >= msg->range_min && range <= msg->range_max) {
        ++point_count;
      }
    }

    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = msg->header.stamp;
    cloud_msg.header.frame_id = target_frame_;

    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(point_count);

    if (point_count == 0U) {
      publisher_->publish(cloud_msg);
      return;
    }

    sensor_msgs::PointCloud2Iterator<float> out_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(cloud_msg, "z");

    for (std::size_t i = 0; i < msg->ranges.size(); ++i) {
      const float range = msg->ranges[i];
      if (!std::isfinite(range) || range < msg->range_min || range > msg->range_max) {
        continue;
      }

      const double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
      *out_x = static_cast<float>(std::cos(angle) * range);
      *out_y = static_cast<float>(std::sin(angle) * range);
      *out_z = static_cast<float>(publish_z_);

      ++out_x;
      ++out_y;
      ++out_z;
    }

    publisher_->publish(cloud_msg);
  }

  std::string scan_topic_;
  std::string cloud_topic_;
  std::string target_frame_;
  double publish_z_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WallScanToPointCloud>());
  rclcpp::shutdown();
  return 0;
}
