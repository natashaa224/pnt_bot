#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <painting_robot_msgs/msg/wall_depth_profile.hpp>
#include <painting_robot_msgs/msg/paint_head_status.hpp>
#include <cmath>
#include <vector>
#include <algorithm>

class WallDepthProfileNode : public rclcpp::Node
{
public:
    WallDepthProfileNode()
    : Node("wall_depth_profile_node"),
      z_min_(0.0),
      z_max_(1.2),
      wall_nominal_dist_(0.75),
      obstacle_threshold_(0.07),
      margin_(0.05),
      gantry_base_z_(0.41),
      lidar_z_ground_(1.35)
    {
        this->declare_parameter("wall_nominal_dist", 0.75);
        this->declare_parameter("obstacle_threshold", 0.07);
        this->declare_parameter("obstacle_margin", 0.05);
        this->declare_parameter("gantry_z_min", 0.0);
        this->declare_parameter("gantry_z_max", 1.20);

        wall_nominal_dist_ = this->get_parameter("wall_nominal_dist").as_double();
        obstacle_threshold_ = this->get_parameter("obstacle_threshold").as_double();
        margin_ = this->get_parameter("obstacle_margin").as_double();
        z_min_ = this->get_parameter("gantry_z_min").as_double();
        z_max_ = this->get_parameter("gantry_z_max").as_double();

        profile_pub_ = this->create_publisher<painting_robot_msgs::msg::WallDepthProfile>(
            "/wall_depth_profile", 10);

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/wall_scan", rclcpp::SensorDataQoS(),
            std::bind(&WallDepthProfileNode::scanCallback, this, std::placeholders::_1));

        head_sub_ = this->create_subscription<painting_robot_msgs::msg::PaintHeadStatus>(
            "/paint_head_status", 10,
            std::bind(&WallDepthProfileNode::headStatusCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "WallDepthProfileNode initialized. Bridging vertical LiDAR to paint_planner.");
    }

private:
    void headStatusCallback(const painting_robot_msgs::msg::PaintHeadStatus::SharedPtr msg)
    {
        if (msg->limit_lower_z && msg->z_soft_upper_mm > msg->z_soft_lower_mm) {
            z_min_ = msg->z_soft_lower_mm / 1000.0;
            z_max_ = msg->z_soft_upper_mm / 1000.0;
        }
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (msg->ranges.empty()) return;

        const int NUM_BINS = 100;
        double stroke_span = std::max(0.2, z_max_ - z_min_);
        double dz = stroke_span / NUM_BINS;

        std::vector<bool> bin_is_obstacle(NUM_BINS, false);
        std::vector<float> bin_depth(NUM_BINS, static_cast<float>(wall_nominal_dist_));

        double angle = msg->angle_min;
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float r = msg->ranges[i];
            double cur_angle = angle;
            angle += msg->angle_increment;

            bool valid = std::isfinite(r) && r >= msg->range_min && r <= 4.5;
            double sin_a = std::sin(cur_angle);
            double cos_a = std::cos(cur_angle);

            // In laser frame pitched -90 deg: wall is in Y < 0 hemisphere (sin_a < 0)
            if (sin_a >= 0.0) continue;

            double z_ground = valid ? (r * cos_a + lidar_z_ground_) : (4.0 * cos_a + lidar_z_ground_);
            double z_stroke = z_ground - gantry_base_z_;

            if (z_stroke < z_min_ || z_stroke > z_max_) continue;

            int bin_idx = static_cast<int>((z_stroke - z_min_) / dz);
            bin_idx = std::max(0, std::min(NUM_BINS - 1, bin_idx));

            if (!valid) {
                // Out-of-bounds beam = window opening / cutout void
                bin_is_obstacle[bin_idx] = true;
                bin_depth[bin_idx] = 999.0f;
                continue;
            }

            double y_dist = std::abs(r * sin_a);
            bin_depth[bin_idx] = static_cast<float>(y_dist);

            // Protrusion obstacle closer to robot than nominal wall
            if (y_dist < (wall_nominal_dist_ - obstacle_threshold_)) {
                bin_is_obstacle[bin_idx] = true;
            }
            // Cutout void further than nominal wall
            else if (y_dist > (wall_nominal_dist_ + 0.12)) {
                bin_is_obstacle[bin_idx] = true;
            }
        }

        // Dilate safety margins around obstacles
        int margin_bins = static_cast<int>(std::ceil(margin_ / dz));
        std::vector<bool> dilated(NUM_BINS, false);
        for (int i = 0; i < NUM_BINS; ++i) {
            if (bin_is_obstacle[i]) {
                int start = std::max(0, i - margin_bins);
                int end = std::min(NUM_BINS - 1, i + margin_bins);
                for (int k = start; k <= end; ++k) {
                    dilated[k] = true;
                }
            }
        }

        // Build WallDepthProfile message
        painting_robot_msgs::msg::WallDepthProfile out_msg;
        out_msg.header.stamp = msg->header.stamp;
        out_msg.header.frame_id = "base_link";
        out_msg.strip_index = 0;
        out_msg.z_min = static_cast<float>(z_min_);
        out_msg.z_max = static_cast<float>(z_max_);

        out_msg.z_positions.reserve(NUM_BINS);
        out_msg.paintable.reserve(NUM_BINS);
        out_msg.depth_mm.reserve(NUM_BINS);

        for (int i = 0; i < NUM_BINS; ++i) {
            float z_sample = static_cast<float>(z_min_ + (i + 0.5) * dz);
            out_msg.z_positions.push_back(z_sample);
            out_msg.paintable.push_back(!dilated[i]);
            out_msg.depth_mm.push_back(bin_depth[i] * 1000.0f);
        }

        profile_pub_->publish(out_msg);
    }

    double z_min_;
    double z_max_;
    double wall_nominal_dist_;
    double obstacle_threshold_;
    double margin_;
    double gantry_base_z_;
    double lidar_z_ground_;

    rclcpp::Publisher<painting_robot_msgs::msg::WallDepthProfile>::SharedPtr profile_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<painting_robot_msgs::msg::PaintHeadStatus>::SharedPtr head_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WallDepthProfileNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
