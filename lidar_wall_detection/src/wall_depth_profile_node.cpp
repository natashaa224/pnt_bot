#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <painting_robot_msgs/msg/wall_depth_profile.hpp>
#include <painting_robot_msgs/msg/paint_head_status.hpp>
#include <std_msgs/msg/string.hpp>
#include <cmath>
#include <vector>
#include <algorithm>
#include <sstream>
#include <iomanip>

class WallDepthProfileNode : public rclcpp::Node
{
public:
    WallDepthProfileNode()
    : Node("wall_depth_profile_node"),
      z_min_(0.0),
      z_max_(1.2),
      wall_nominal_dist_(0.41),
      obstacle_threshold_(0.025),
      margin_(0.05),
      gantry_base_z_(0.410),
      lidar_z_ground_(1.460),
      min_cutout_consecutive_rays_(3),
      min_protrusion_consecutive_rays_(2)
    {
        this->declare_parameter("wall_nominal_dist", 0.41);
        this->declare_parameter("obstacle_threshold", 0.025);
        this->declare_parameter("obstacle_margin", 0.05);
        this->declare_parameter("gantry_z_min", 0.0);
        this->declare_parameter("gantry_z_max", 1.20);
        this->declare_parameter("gantry_base_z", 0.410);
        this->declare_parameter("lidar_z_ground", 1.460);
        this->declare_parameter("min_cutout_consecutive_rays", 3);
        this->declare_parameter("min_protrusion_consecutive_rays", 2);

        wall_nominal_dist_ = this->get_parameter("wall_nominal_dist").as_double();
        obstacle_threshold_ = this->get_parameter("obstacle_threshold").as_double();
        margin_ = this->get_parameter("obstacle_margin").as_double();
        z_min_ = this->get_parameter("gantry_z_min").as_double();
        z_max_ = this->get_parameter("gantry_z_max").as_double();
        gantry_base_z_ = this->get_parameter("gantry_base_z").as_double();
        lidar_z_ground_ = this->get_parameter("lidar_z_ground").as_double();
        min_cutout_consecutive_rays_ = this->get_parameter("min_cutout_consecutive_rays").as_int();
        min_protrusion_consecutive_rays_ = this->get_parameter("min_protrusion_consecutive_rays").as_int();

        profile_pub_ = this->create_publisher<painting_robot_msgs::msg::WallDepthProfile>(
            "/wall_depth_profile", 10);

        summary_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/wall_obstacle_summary", 10);

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/wall_scan", rclcpp::SensorDataQoS(),
            std::bind(&WallDepthProfileNode::scanCallback, this, std::placeholders::_1));

        head_sub_ = this->create_subscription<painting_robot_msgs::msg::PaintHeadStatus>(
            "/paint_head_status", 10,
            std::bind(&WallDepthProfileNode::headStatusCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
            "WallDepthProfileNode initialized (thresh=%.3fm, margin=%.3fm, lidar_z=%.3fm, gantry_z=%.3fm).",
            obstacle_threshold_, margin_, lidar_z_ground_, gantry_base_z_);
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

        // Pass 1: Dynamically determine flat wall surface distance using median of valid beams
        std::vector<double> valid_y;
        valid_y.reserve(msg->ranges.size());
        double angle = msg->angle_min;
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float r = msg->ranges[i];
            double cur_angle = angle;
            angle += msg->angle_increment;

            if (!std::isfinite(r) || r < msg->range_min || r > 4.5) continue;
            double sin_a = std::sin(cur_angle);
            if (sin_a >= -0.05) continue; // In laser frame: wall is in Y < 0 hemisphere

            double y_dist = std::abs(r * sin_a);
            if (y_dist >= 0.20 && y_dist <= 1.50) {
                valid_y.push_back(y_dist);
            }
        }

        double d_nominal = wall_nominal_dist_;
        if (valid_y.size() >= 15) {
            size_t mid_idx = valid_y.size() / 2;
            std::nth_element(valid_y.begin(), valid_y.begin() + mid_idx, valid_y.end());
            d_nominal = valid_y[mid_idx];
        }

        // Pass 2: Pre-filter rays & classify with confirmation logic (Test 6)
        // Group consecutive invalid/cutout rays and consecutive protrusion rays
        const size_t num_rays = msg->ranges.size();
        std::vector<bool> ray_is_cutout(num_rays, false);
        std::vector<bool> ray_is_protrusion(num_rays, false);
        std::vector<double> ray_y_dist(num_rays, d_nominal);
        std::vector<double> ray_z_stroke(num_rays, -999.0);

        angle = msg->angle_min;
        for (size_t i = 0; i < num_rays; ++i) {
            float r = msg->ranges[i];
            double cur_angle = angle;
            angle += msg->angle_increment;

            double sin_a = std::sin(cur_angle);
            double cos_a = std::cos(cur_angle);

            // Wall is in sin_a < -0.05
            if (sin_a >= -0.05) continue;

            bool valid = std::isfinite(r) && r >= msg->range_min && r <= 4.5;
            double z_ground = valid ? (r * cos_a + lidar_z_ground_) : (4.0 * cos_a + lidar_z_ground_);
            double z_stroke = z_ground - gantry_base_z_;
            ray_z_stroke[i] = z_stroke;

            if (!valid) {
                ray_is_cutout[i] = true;
                ray_y_dist[i] = 999.0;
            } else {
                double y_dist = std::abs(r * sin_a);
                ray_y_dist[i] = y_dist;

                if (y_dist < (d_nominal - obstacle_threshold_)) {
                    ray_is_protrusion[i] = true;
                } else if (y_dist > (d_nominal + 0.08)) {
                    ray_is_cutout[i] = true;
                }
            }
        }

        // Confirmation filter (Test 6: Do not classify isolated dropped rays as confirmed cutouts)
        std::vector<bool> confirmed_cutout(num_rays, false);
        std::vector<bool> confirmed_protrusion(num_rays, false);

        // Filter cutouts: require at least min_cutout_consecutive_rays_
        size_t c_start = 0;
        while (c_start < num_rays) {
            if (!ray_is_cutout[c_start]) {
                ++c_start;
                continue;
            }
            size_t c_end = c_start;
            while (c_end < num_rays && ray_is_cutout[c_end]) {
                ++c_end;
            }
            int run_len = static_cast<int>(c_end - c_start);
            if (run_len >= min_cutout_consecutive_rays_) {
                for (size_t k = c_start; k < c_end; ++k) {
                    confirmed_cutout[k] = true;
                }
            }
            c_start = c_end;
        }

        // Filter protrusions: require at least min_protrusion_consecutive_rays_
        size_t p_start = 0;
        while (p_start < num_rays) {
            if (!ray_is_protrusion[p_start]) {
                ++p_start;
                continue;
            }
            size_t p_end = p_start;
            while (p_end < num_rays && ray_is_protrusion[p_end]) {
                ++p_end;
            }
            int run_len = static_cast<int>(p_end - p_start);
            if (run_len >= min_protrusion_consecutive_rays_) {
                for (size_t k = p_start; k < p_end; ++k) {
                    confirmed_protrusion[k] = true;
                }
            }
            p_start = p_end;
        }

        // Pass 3: Map confirmed rays into 100 vertical stroke bins
        const int NUM_BINS = 100;
        double stroke_span = std::max(0.2, z_max_ - z_min_);
        double dz = stroke_span / NUM_BINS;

        std::vector<bool> bin_is_obstacle(NUM_BINS, false);
        std::vector<float> bin_depth(NUM_BINS, static_cast<float>(d_nominal));
        std::vector<std::string> bin_type(NUM_BINS, "CLEAR");

        int cutout_bins_count = 0;
        int protrusion_bins_count = 0;
        double obs_z_min = 999.0;
        double obs_z_max = -999.0;

        for (size_t i = 0; i < num_rays; ++i) {
            double z_stroke = ray_z_stroke[i];
            if (z_stroke < z_min_ || z_stroke > z_max_) continue;

            int bin_idx = static_cast<int>((z_stroke - z_min_) / dz);
            bin_idx = std::max(0, std::min(NUM_BINS - 1, bin_idx));

            if (confirmed_cutout[i]) {
                bin_is_obstacle[bin_idx] = true;
                bin_depth[bin_idx] = static_cast<float>(ray_y_dist[i]);
                bin_type[bin_idx] = "CUTOUT";
                cutout_bins_count++;
                obs_z_min = std::min(obs_z_min, z_stroke);
                obs_z_max = std::max(obs_z_max, z_stroke);
            } else if (confirmed_protrusion[i]) {
                bin_is_obstacle[bin_idx] = true;
                bin_depth[bin_idx] = static_cast<float>(ray_y_dist[i]);
                bin_type[bin_idx] = "PROTRUSION";
                protrusion_bins_count++;
                obs_z_min = std::min(obs_z_min, z_stroke);
                obs_z_max = std::max(obs_z_max, z_stroke);
            } else {
                bin_depth[bin_idx] = static_cast<float>(ray_y_dist[i]);
            }
        }

        // Dilate safety margins around detected obstacles
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

        // Publish obstacle summary for visual HUD and metrics tracker
        bool has_obstacle = (cutout_bins_count > 0 || protrusion_bins_count > 0);
        std::string obs_class = "CLEAR";
        if (cutout_bins_count > 0 && protrusion_bins_count > 0) {
            obs_class = "MULTIPLE_OBSTACLES";
        } else if (cutout_bins_count > 0) {
            obs_class = (obs_z_min < 0.25) ? "DOORWAY" : "WINDOW";
        } else if (protrusion_bins_count > 0) {
            double span = obs_z_max - obs_z_min;
            if (span > 0.40) {
                obs_class = "CONDUIT_OR_COLUMN";
            } else if (span > 0.18) {
                obs_class = "ELECTRICAL_BOX";
            } else if (span > 0.11) {
                obs_class = "SWITCHBOARD";
            } else {
                obs_class = "MINOR_PROTRUSION";
            }
        }

        std::ostringstream ss;
        ss << std::fixed << std::setprecision(3);
        ss << "{"
           << "\"detected\":" << (has_obstacle ? "true" : "false") << ","
           << "\"classification\":\"" << obs_class << "\","
           << "\"d_nominal\":" << d_nominal << ","
           << "\"z_min\":" << (has_obstacle ? obs_z_min : 0.0) << ","
           << "\"z_max\":" << (has_obstacle ? obs_z_max : 0.0) << ","
           << "\"margin\":" << margin_ << ","
           << "\"cutout_bins\":" << cutout_bins_count << ","
           << "\"protrusion_bins\":" << protrusion_bins_count
           << "}";

        std_msgs::msg::String sum_msg;
        sum_msg.data = ss.str();
        summary_pub_->publish(sum_msg);
    }

    double z_min_;
    double z_max_;
    double wall_nominal_dist_;
    double obstacle_threshold_;
    double margin_;
    double gantry_base_z_;
    double lidar_z_ground_;
    int min_cutout_consecutive_rays_;
    int min_protrusion_consecutive_rays_;

    rclcpp::Publisher<painting_robot_msgs::msg::WallDepthProfile>::SharedPtr profile_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr summary_pub_;
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
