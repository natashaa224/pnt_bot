#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/string.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class CutoutWindowDetector : public rclcpp::Node
{
public:
    CutoutWindowDetector()
        : Node("cutout_window_detector"),
          has_tf_(false)
    {
        // ============================================================
        // PARAMETERS
        // ============================================================
        this->declare_parameter<std::string>("input_scan_topic", "/wall_scan_filtered");
        this->declare_parameter<std::string>("fixed_frame", "odom");
        this->declare_parameter<std::string>("lidar_frame", "laser");

        // Baseline wall location (distance from robot/laser to wall)
        this->declare_parameter<double>("baseline_y", -0.750);
        this->declare_parameter<double>("baseline_tolerance", 0.035);

        // Threshold beyond which a ray is considered a void/cutout (in meters)
        this->declare_parameter<double>("cutout_depth_threshold", 0.08);

        // Geometry constraints for a valid window/cutout
        this->declare_parameter<double>("min_cutout_height", 0.20);
        this->declare_parameter<double>("max_cutout_height", 2.40);
        this->declare_parameter<int>("min_consecutive_gap_rays", 4);
        this->declare_parameter<int>("min_wall_boundary_rays", 3);

        // Sill height threshold to distinguish Window from Doorway (meters above ground)
        this->declare_parameter<double>("doorway_sill_threshold", 0.25);

        // LiDAR mounting height above ground
        this->declare_parameter<double>("lidar_height_above_ground", 0.60);

        // Laser scan limits
        this->declare_parameter<double>("min_range", 0.08);
        this->declare_parameter<double>("max_range", 16.0);

        // Read parameters
        input_scan_topic_ = this->get_parameter("input_scan_topic").as_string();
        fixed_frame_ = this->get_parameter("fixed_frame").as_string();
        lidar_frame_ = this->get_parameter("lidar_frame").as_string();
        baseline_y_ = this->get_parameter("baseline_y").as_double();
        baseline_tolerance_ = this->get_parameter("baseline_tolerance").as_double();
        cutout_depth_threshold_ = this->get_parameter("cutout_depth_threshold").as_double();
        min_cutout_height_ = this->get_parameter("min_cutout_height").as_double();
        max_cutout_height_ = this->get_parameter("max_cutout_height").as_double();
        min_consecutive_gap_rays_ = this->get_parameter("min_consecutive_gap_rays").as_int();
        min_wall_boundary_rays_ = this->get_parameter("min_wall_boundary_rays").as_int();
        doorway_sill_threshold_ = this->get_parameter("doorway_sill_threshold").as_double();
        lidar_height_above_ground_ = this->get_parameter("lidar_height_above_ground").as_double();
        min_range_ = this->get_parameter("min_range").as_double();
        max_range_ = this->get_parameter("max_range").as_double();

        // ============================================================
        // TF LISTENER
        // ============================================================
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // ============================================================
        // SUBSCRIBER
        // ============================================================
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            input_scan_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&CutoutWindowDetector::scanCallback, this, std::placeholders::_1));

        // ============================================================
        // PUBLISHERS
        // ============================================================
        // 3D Bounding Boxes for detected windows/cutouts (cyan box)
        cutout_box_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/detected_cutout_box", 10);

        // Wireframe boundary outline and sill/lintel frames
        boundary_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/cutout_boundary_markers", 10);

        // 3D Text labels displaying dimensions and classification
        label_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/cutout_text_labels", 10);

        // Real-time slice markers on the active laser slice
        slice_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/cutout_slice_markers", 10);

        // Status text / JSON publisher
        status_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/wall_cutout_status", 10);

        RCLCPP_INFO(this->get_logger(), "==========================================");
        RCLCPP_INFO(this->get_logger(), "    WALL CUTOUT & WINDOW DETECTOR STARTED");
        RCLCPP_INFO(this->get_logger(), "==========================================");
        RCLCPP_INFO(this->get_logger(), "Input Topic        : %s", input_scan_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Fixed Frame        : %s", fixed_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Baseline Y         : %.3f m", baseline_y_);
        RCLCPP_INFO(this->get_logger(), "Depth Threshold    : %.3f m", cutout_depth_threshold_);
        RCLCPP_INFO(this->get_logger(), "Min Cutout Height  : %.3f m", min_cutout_height_);
        RCLCPP_INFO(this->get_logger(), "LiDAR Height (Z)   : %.3f m", lidar_height_above_ground_);
        RCLCPP_INFO(this->get_logger(), "==========================================");
    }

private:
    enum class RayClass
    {
        INVALID,
        WALL,
        PROTRUSION,
        VOID_CUTOUT
    };

    struct VerticalCutoutSlice
    {
        double sill_laser_x;     // Bottom boundary in laser frame (x_laser)
        double lintel_laser_x;   // Top boundary in laser frame (x_laser)
        double sill_ground_z;    // Sill height above ground (meters)
        double lintel_ground_z;  // Lintel height above ground (meters)
        double height;           // Opening height (meters)
        double wall_distance_y;  // Perpendicular distance to wall
        std::string classification; // "WINDOW", "DOORWAY", or "RECESSED_NICHE"
        int ray_count;
    };

    struct TrackedCutout
    {
        int id;
        double min_x;            // Along-wall X extent (world coords)
        double max_x;
        double wall_y;           // Wall Y position in world coords
        double sill_z;           // Sill height above ground (world coords)
        double lintel_z;         // Lintel height above ground (world coords)
        double width;            // Width = max_x - min_x
        double height;           // Height = lintel_z - sill_z
        double center_x;
        double center_y;
        double center_z;
        std::string classification;
        int observations;
        rclcpp::Time first_seen;
        rclcpp::Time last_seen;
        bool finalized;
    };

    // ================================================================
    // SCAN CALLBACK
    // ================================================================
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (msg->ranges.empty()) {
            return;
        }

        // Look up robot pose in fixed_frame (odom)
        double robot_x = 0.0;
        double robot_y = 0.0;
        bool has_robot_pose = getRobotPose(robot_x, robot_y, msg->header.stamp);

        // 1. Classify each laser ray
        std::vector<RayClass> ray_classes(msg->ranges.size(), RayClass::INVALID);
        std::vector<double> ray_laser_x(msg->ranges.size(), 0.0);
        std::vector<double> ray_laser_y(msg->ranges.size(), 0.0);
        std::vector<double> ray_expected_range(msg->ranges.size(), 0.0);

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
            double sin_angle = std::sin(angle);
            double cos_angle = std::cos(angle);

            // Wall is in the direction where sin(angle) < 0
            if (sin_angle >= -0.05) {
                continue;
            }

            // Expected baseline distance to the flat wall
            double expected_range = baseline_y_ / sin_angle;
            if (!std::isfinite(expected_range) || expected_range <= 0.0) {
                continue;
            }

            ray_expected_range[i] = expected_range;
            double r = static_cast<double>(msg->ranges[i]);

            // Check if ray is infinite / no return or passes far beyond wall
            bool is_missing_or_deep = (!std::isfinite(r)) ||
                                      (r > max_range_) ||
                                      (r > (expected_range + cutout_depth_threshold_));

            if (is_missing_or_deep) {
                ray_classes[i] = RayClass::VOID_CUTOUT;
                // Position where beam passes through the wall plane
                ray_laser_x[i] = expected_range * cos_angle;
                ray_laser_y[i] = baseline_y_;
            } else if (r >= min_range_) {
                double x = r * cos_angle;
                double y = r * sin_angle;
                ray_laser_x[i] = x;
                ray_laser_y[i] = y;

                double diff = expected_range - r;
                if (std::abs(y - baseline_y_) <= baseline_tolerance_ || std::abs(diff) <= baseline_tolerance_) {
                    ray_classes[i] = RayClass::WALL;
                } else if (diff > 0.04) {
                    ray_classes[i] = RayClass::PROTRUSION;
                } else {
                    ray_classes[i] = RayClass::VOID_CUTOUT;
                }
            }
        }

        // 2. Detect vertical cutout spans bounded by wall points
        std::vector<VerticalCutoutSlice> detected_slices;
        findCutoutSpans(ray_classes, ray_laser_x, ray_laser_y, *msg, detected_slices);

        // 3. Publish real-time slice markers in the laser frame
        publishSliceMarkers(msg->header.frame_id, detected_slices);

        // 4. Update 3D tracking if robot pose is available
        if (has_robot_pose) {
            update3DTracking(robot_x, robot_y, detected_slices, msg->header.stamp);
            publish3DCutoutVisualizations();
        }
    }

    // ================================================================
    // FIND CUTOUT SPANS IN CURRENT VERTICAL SLICE
    // ================================================================
    void findCutoutSpans(
        const std::vector<RayClass> & ray_classes,
        const std::vector<double> & ray_laser_x,
        const std::vector<double> & ray_laser_y,
        const sensor_msgs::msg::LaserScan & msg,
        std::vector<VerticalCutoutSlice> & detected_slices)
    {
        (void)ray_laser_y;
        (void)msg;
        size_t n = ray_classes.size();
        size_t i = 0;

        while (i < n) {
            if (ray_classes[i] != RayClass::VOID_CUTOUT) {
                ++i;
                continue;
            }

            size_t gap_start = i;
            while (i < n && ray_classes[i] == RayClass::VOID_CUTOUT) {
                ++i;
            }
            size_t gap_end = i - 1;
            int gap_count = static_cast<int>(gap_end - gap_start + 1);

            if (gap_count < min_consecutive_gap_rays_) {
                continue;
            }

            // Check boundary rays before gap
            int wall_before = 0;
            size_t k = (gap_start > 0) ? (gap_start - 1) : 0;
            while (k > 0 && wall_before < min_wall_boundary_rays_) {
                if (ray_classes[k] == RayClass::WALL) {
                    ++wall_before;
                } else if (ray_classes[k] == RayClass::PROTRUSION) {
                    // Obstacle adjacent to window is also acceptable boundary
                    ++wall_before;
                } else {
                    break;
                }
                --k;
            }

            // Check boundary rays after gap
            int wall_after = 0;
            k = gap_end + 1;
            while (k < n && wall_after < min_wall_boundary_rays_) {
                if (ray_classes[k] == RayClass::WALL) {
                    ++wall_after;
                } else if (ray_classes[k] == RayClass::PROTRUSION) {
                    ++wall_after;
                } else {
                    break;
                }
                ++k;
            }

            // In laser frame: x_laser points UP (+Z in base_link)
            // Points sorted along the opening:
            double x_start = ray_laser_x[gap_start];
            double x_end = ray_laser_x[gap_end];

            double min_x = std::min(x_start, x_end);
            double max_x = std::max(x_start, x_end);

            double sill_ground_z = lidar_height_above_ground_ + min_x;
            double lintel_ground_z = lidar_height_above_ground_ + max_x;
            double height = lintel_ground_z - sill_ground_z;

            if (height < min_cutout_height_ || height > max_cutout_height_) {
                continue;
            }

            // Valid boundary conditions:
            // 1. Both walls present -> WINDOW / WALL CUTOUT
            // 2. Upper wall present, bottom extends to floor -> DOORWAY
            bool is_window = (wall_before >= min_wall_boundary_rays_ && wall_after >= min_wall_boundary_rays_);
            bool is_doorway = (wall_after >= min_wall_boundary_rays_ || wall_before >= min_wall_boundary_rays_) &&
                              (sill_ground_z <= doorway_sill_threshold_);

            if (!is_window && !is_doorway) {
                continue;
            }

            VerticalCutoutSlice slice;
            slice.sill_laser_x = min_x;
            slice.lintel_laser_x = max_x;
            slice.sill_ground_z = sill_ground_z;
            slice.lintel_ground_z = lintel_ground_z;
            slice.height = height;
            slice.wall_distance_y = baseline_y_;
            slice.ray_count = gap_count;

            if (sill_ground_z > doorway_sill_threshold_) {
                slice.classification = "WINDOW";
            } else {
                slice.classification = "DOORWAY";
            }

            detected_slices.push_back(slice);

            RCLCPP_DEBUG(this->get_logger(),
                "Cutout slice: %s | Sill: %.2fm -> Lintel: %.2fm (Height: %.2fm) [%d rays]",
                slice.classification.c_str(), slice.sill_ground_z, slice.lintel_ground_z,
                slice.height, slice.ray_count);
        }
    }

    // ================================================================
    // GET ROBOT POSE VIA TF
    // ================================================================
    bool getRobotPose(double & x, double & y, const rclcpp::Time & stamp)
    {
        try {
            geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
                fixed_frame_, lidar_frame_, stamp, std::chrono::milliseconds(50));
            x = tf.transform.translation.x;
            y = tf.transform.translation.y;
            has_tf_ = true;
            return true;
        } catch (const tf2::TransformException &) {
            // Fallback to latest available transform
            try {
                geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
                    fixed_frame_, lidar_frame_, tf2::TimePointZero);
                x = tf.transform.translation.x;
                y = tf.transform.translation.y;
                has_tf_ = true;
                return true;
            } catch (const tf2::TransformException &) {
                return false;
            }
        }
    }

    // ================================================================
    // 3D SPATIAL TRACKING ACROSS ROBOT MOTION
    // ================================================================
    void update3DTracking(
        double robot_x,
        double robot_y,
        const std::vector<VerticalCutoutSlice> & slices,
        const rclcpp::Time & stamp)
    {
        double wall_world_y = robot_y + baseline_y_;

        if (!slices.empty()) {
            const auto & s = slices.front();

            // Check if matches an existing active cutout nearby
            bool matched = false;
            for (auto & cutout : active_cutouts_) {
                double dist_x = std::abs(robot_x - cutout.center_x);
                if (dist_x < 1.2 && std::abs(s.sill_ground_z - cutout.sill_z) < 0.40) {
                    // Update bounds
                    cutout.min_x = std::min(cutout.min_x, robot_x);
                    cutout.max_x = std::max(cutout.max_x, robot_x);
                    cutout.sill_z = std::min(cutout.sill_z, s.sill_ground_z);
                    cutout.lintel_z = std::max(cutout.lintel_z, s.lintel_ground_z);
                    cutout.width = std::max(0.10, cutout.max_x - cutout.min_x + 0.15);
                    cutout.height = cutout.lintel_z - cutout.sill_z;
                    cutout.center_x = (cutout.min_x + cutout.max_x) * 0.5;
                    cutout.center_y = wall_world_y;
                    cutout.center_z = (cutout.sill_z + cutout.lintel_z) * 0.5;
                    cutout.last_seen = stamp;
                    cutout.observations++;
                    matched = true;
                    break;
                }
            }

            if (!matched) {
                TrackedCutout new_cutout;
                new_cutout.id = static_cast<int>(active_cutouts_.size() + 1);
                new_cutout.min_x = robot_x - 0.05;
                new_cutout.max_x = robot_x + 0.05;
                new_cutout.wall_y = wall_world_y;
                new_cutout.sill_z = s.sill_ground_z;
                new_cutout.lintel_z = s.lintel_ground_z;
                new_cutout.width = 0.20;
                new_cutout.height = s.height;
                new_cutout.center_x = robot_x;
                new_cutout.center_y = wall_world_y;
                new_cutout.center_z = (s.sill_ground_z + s.lintel_ground_z) * 0.5;
                new_cutout.classification = s.classification;
                new_cutout.observations = 1;
                new_cutout.first_seen = stamp;
                new_cutout.last_seen = stamp;
                new_cutout.finalized = false;
                active_cutouts_.push_back(new_cutout);

                RCLCPP_INFO(this->get_logger(),
                    "[NEW %s DETECTED] at X=%.2fm | Sill: %.2fm, Lintel: %.2fm, Height: %.2fm",
                    new_cutout.classification.c_str(), robot_x,
                    new_cutout.sill_z, new_cutout.lintel_z, new_cutout.height);
            }
        }

        // Publish status text message
        if (!active_cutouts_.empty()) {
            std::stringstream ss;
            ss << "Detected Cutouts/Windows (" << active_cutouts_.size() << "): ";
            for (const auto & c : active_cutouts_) {
                ss << "[" << c.classification << " #" << c.id << " at X=" << std::fixed
                   << std::setprecision(2) << c.center_x << "m: " << c.width << "m(W) x "
                   << c.height << "m(H), Sill=" << c.sill_z << "m] ";
            }
            std_msgs::msg::String status_msg;
            status_msg.data = ss.str();
            status_pub_->publish(status_msg);
        }
    }

    // ================================================================
    // PUBLISH REAL-TIME SLICE MARKERS (Laser Frame)
    // ================================================================
    void publishSliceMarkers(
        const std::string & frame_id,
        const std::vector<VerticalCutoutSlice> & slices)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = this->now();
        marker.ns = "cutout_slices";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.04; // Line width

        if (slices.empty()) {
            marker.action = visualization_msgs::msg::Marker::DELETE;
            slice_marker_pub_->publish(marker);
            return;
        }

        // Bright Cyan color for Cutout Slice lines
        marker.color.r = 0.0F;
        marker.color.g = 0.9F;
        marker.color.b = 1.0F;
        marker.color.a = 0.95F;

        for (const auto & s : slices) {
            // Vertical opening segment
            geometry_msgs::msg::Point p1, p2;
            p1.x = s.sill_laser_x;
            p1.y = s.wall_distance_y;
            p1.z = 0.0;
            p2.x = s.lintel_laser_x;
            p2.y = s.wall_distance_y;
            p2.z = 0.0;
            marker.points.push_back(p1);
            marker.points.push_back(p2);

            // Sill cross-tick (Yellow-green)
            geometry_msgs::msg::Point s1, s2;
            s1.x = s.sill_laser_x;
            s1.y = s.wall_distance_y - 0.08;
            s1.z = 0.0;
            s2.x = s.sill_laser_x;
            s2.y = s.wall_distance_y + 0.08;
            s2.z = 0.0;
            marker.points.push_back(s1);
            marker.points.push_back(s2);

            // Lintel cross-tick
            geometry_msgs::msg::Point l1, l2;
            l1.x = s.lintel_laser_x;
            l1.y = s.wall_distance_y - 0.08;
            l1.z = 0.0;
            l2.x = s.lintel_laser_x;
            l2.y = s.wall_distance_y + 0.08;
            l2.z = 0.0;
            marker.points.push_back(l1);
            marker.points.push_back(l2);
        }

        slice_marker_pub_->publish(marker);
    }

    // ================================================================
    // PUBLISH 3D RECONSTRUCTED CUTOUT VISUALIZATIONS (Fixed Frame)
    // ================================================================
    void publish3DCutoutVisualizations()
    {
        visualization_msgs::msg::MarkerArray box_array;
        visualization_msgs::msg::MarkerArray boundary_array;
        visualization_msgs::msg::MarkerArray label_array;

        rclcpp::Time now = this->now();

        for (size_t i = 0; i < active_cutouts_.size(); ++i) {
            const auto & c = active_cutouts_[i];

            // 1. Semi-transparent Cyan 3D Bounding Box
            visualization_msgs::msg::Marker box;
            box.header.frame_id = fixed_frame_;
            box.header.stamp = now;
            box.ns = "cutout_3d_boxes";
            box.id = static_cast<int>(i);
            box.type = visualization_msgs::msg::Marker::CUBE;
            box.action = visualization_msgs::msg::Marker::ADD;
            box.pose.position.x = c.center_x;
            box.pose.position.y = c.center_y;
            box.pose.position.z = c.center_z;
            box.pose.orientation.w = 1.0;
            box.scale.x = std::max(0.15, c.width);
            box.scale.y = 0.22; // Wall thickness + margin
            box.scale.z = std::max(0.15, c.height);

            // Cyan fill with gentle transparency
            box.color.r = 0.0F;
            box.color.g = 0.85F;
            box.color.b = 1.0F;
            box.color.a = 0.40F;
            box_array.markers.push_back(box);

            // 2. Wireframe Boundary Lines (Sill, Lintel, Left/Right Jambs)
            visualization_msgs::msg::Marker frame_lines;
            frame_lines.header.frame_id = fixed_frame_;
            frame_lines.header.stamp = now;
            frame_lines.ns = "cutout_frame_lines";
            frame_lines.id = static_cast<int>(i);
            frame_lines.type = visualization_msgs::msg::Marker::LINE_STRIP;
            frame_lines.action = visualization_msgs::msg::Marker::ADD;
            frame_lines.scale.x = 0.035; // Wire width
            frame_lines.color.r = 0.1F;
            frame_lines.color.g = 1.0F;
            frame_lines.color.b = 0.9F;
            frame_lines.color.a = 1.0F;
            frame_lines.pose.orientation.w = 1.0;

            double half_w = std::max(0.10, c.width * 0.5);
            double x1 = c.center_x - half_w;
            double x2 = c.center_x + half_w;
            double z1 = c.sill_z;
            double z2 = c.lintel_z;
            double y_face = c.center_y + 0.11;

            geometry_msgs::msg::Point p_bl, p_br, p_tr, p_tl;
            p_bl.x = x1; p_bl.y = y_face; p_bl.z = z1;
            p_br.x = x2; p_br.y = y_face; p_br.z = z1;
            p_tr.x = x2; p_tr.y = y_face; p_tr.z = z2;
            p_tl.x = x1; p_tl.y = y_face; p_tl.z = z2;

            frame_lines.points.push_back(p_bl);
            frame_lines.points.push_back(p_br);
            frame_lines.points.push_back(p_tr);
            frame_lines.points.push_back(p_tl);
            frame_lines.points.push_back(p_bl); // Close loop
            boundary_array.markers.push_back(frame_lines);

            // 3. Floating 3D Text Label
            visualization_msgs::msg::Marker text_marker;
            text_marker.header.frame_id = fixed_frame_;
            text_marker.header.stamp = now;
            text_marker.ns = "cutout_labels";
            text_marker.id = static_cast<int>(i);
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;
            text_marker.pose.position.x = c.center_x;
            text_marker.pose.position.y = c.center_y + 0.15;
            text_marker.pose.position.z = c.lintel_z + 0.22;
            text_marker.pose.orientation.w = 1.0;
            text_marker.scale.z = 0.16; // Text height
            text_marker.color.r = 0.1F;
            text_marker.color.g = 1.0F;
            text_marker.color.b = 0.9F;
            text_marker.color.a = 1.0F;

            std::stringstream ss;
            ss << "[" << c.classification << "]\n"
               << "W: " << std::fixed << std::setprecision(2) << c.width << "m x H: " << c.height << "m\n"
               << "Sill: " << c.sill_z << "m | Lintel: " << c.lintel_z << "m";
            text_marker.text = ss.str();
            label_array.markers.push_back(text_marker);
        }

        cutout_box_pub_->publish(box_array);
        boundary_marker_pub_->publish(boundary_array);
        label_pub_->publish(label_array);
    }

    // Members
    std::string input_scan_topic_;
    std::string fixed_frame_;
    std::string lidar_frame_;
    double baseline_y_;
    double baseline_tolerance_;
    double cutout_depth_threshold_;
    double min_cutout_height_;
    double max_cutout_height_;
    int min_consecutive_gap_rays_;
    int min_wall_boundary_rays_;
    double doorway_sill_threshold_;
    double lidar_height_above_ground_;
    double min_range_;
    double max_range_;

    bool has_tf_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cutout_box_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr boundary_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr label_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr slice_marker_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

    std::vector<TrackedCutout> active_cutouts_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CutoutWindowDetector>());
    rclcpp::shutdown();
    return 0;
}
