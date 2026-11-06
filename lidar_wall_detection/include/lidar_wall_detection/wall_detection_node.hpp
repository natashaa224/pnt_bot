#pragma once

#include <chrono>
#include <cmath>
#include <memory>
#include <vector>
#include <string>
#include <algorithm>
#include <random>
#include <optional>
#include <tuple>

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "painting_robot_msgs/msg/wall_pose.hpp"
#include "painting_robot_msgs/msg/wall_status.hpp"

namespace lidar_wall_detection {

// ── Classification targets (robot frame, normal pointing toward robot) ────────
static constexpr double RIGHT_WALL_THETA = M_PI / 2.0;    // +90° — right wall normal points +Y
static constexpr double FRONT_WALL_THETA = M_PI;          // ±180° — front wall normal points -X

// ── Tuning constants ──────────────────────────────────────────────────────────
static constexpr int    RANSAC_ITERATIONS  = 100;
static constexpr double RANSAC_THRESHOLD   = 0.05;        // m — point-to-line inlier distance
static constexpr int    RANSAC_MIN_INLIERS = 8;           // minimum inliers for a valid line
static constexpr double MIN_WALL_LENGTH    = 0.3;         // m — reject very short detections
static constexpr double EMA_ALPHA          = 0.3;         // smoothing — lower = smoother/slower
static constexpr int    MAX_MISSED_FRAMES  = 5;           // frames before invalidating lost wall

// ── LiDAR mounting ────────────────────────────────────────────────────────────
static constexpr double LIDAR_YAW_OFFSET   = -M_PI;       // Rotates scan so arrow->code 0° (front), right wall->270°

// ── Angles and Range Targets ──────────────────────────────────────────────────
static constexpr double FRONT_TARGET_DEG   = 180.0;
static constexpr double RIGHT_TARGET_DEG   = 90.0;
static constexpr double THETA_TOL_DEG      = 30.0;
static constexpr double RANGE_TOL          = 0.25;        // 25% relative tolerance
static constexpr double MIN_RANGE_DEFAULT  = 0.15;
static constexpr double MAX_RANGE_DEFAULT  = 15.0;
static constexpr double RIGHT_MAX_DIST_DEF = 1.75;
static constexpr double RIGHT_CENTER_DEF   = 275.0;
static constexpr double RIGHT_SPREAD_DEF   = 15.0;
static constexpr double FRONT_CONE_HALF_DEF= 30.0;
static constexpr double FRONT_MIN_VALID_DEF= 0.05;

inline double angle_diff(double a, double b) {
    double d = a - b;
    while (d > M_PI)  d -= 2.0 * M_PI;
    while (d < -M_PI) d += 2.0 * M_PI;
    return d;
}

inline double angle_diff_deg(double a, double b) {
    double d = a - b;
    while (d > 180.0)  d -= 360.0;
    while (d < -180.0) d += 360.0;
    return d;
}

// ── Geometric Fit Result Structures ───────────────────────────────────────────
struct RansacResult {
    std::vector<bool> inlier_mask;
    Eigen::Vector2d   normal;
    double            d = 0.0;
    int               inlier_count = 0;
};

struct PcaResult {
    Eigen::Vector2d normal;
    double          distance = 0.0;
    Eigen::Vector2d centroid;
    Eigen::Vector2d eigenvalues;
    double          ratio = 0.0;
};

struct WallFit {
    double          ratio = 0.0;
    double          distance = 0.0;
    double          theta = 0.0;
    Eigen::Vector2d normal;
};

struct EmaState {
    double distance = 0.0;
    double theta = 0.0;
};

struct RangeCheckResult {
    double      median_r = 0.0;
    bool        passed = false;
    std::string reason;
};

struct ThetaCheckResult {
    double      theta_deg = 0.0;
    bool        passed = false;
    std::string reason;
};

// ── Geometry Algorithms ───────────────────────────────────────────────────────
std::optional<RansacResult> ransac_line(
    const std::vector<Eigen::Vector2d>& points,
    int n_iter = RANSAC_ITERATIONS,
    double threshold = RANSAC_THRESHOLD,
    int min_inliers = RANSAC_MIN_INLIERS);

std::optional<PcaResult> pca_fit(const std::vector<Eigen::Vector2d>& arc_points);

RangeCheckResult range_check(
    const std::vector<float>& ranges,
    const std::vector<double>& angles_deg,
    double center_deg,
    double window_deg = 5.0,
    std::optional<double> max_dist = std::nullopt,
    std::optional<double> fitted_dist = std::nullopt,
    double rel_tolerance = 0.25);

ThetaCheckResult theta_check(const PcaResult& pca_result, double target_deg, double tolerance_deg = 30.0);

// ── WallDetectionNode Class ───────────────────────────────────────────────────
class WallDetectionNode : public rclcpp::Node {
public:
    explicit WallDetectionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    virtual ~WallDetectionNode() = default;

    // Public getters for programmatic usage
    std::optional<EmaState> get_right_wall_state() const { return right_state_; }
    std::string get_robot_state() const { return robot_state_; }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void state_callback(const std_msgs::msg::String::SharedPtr msg);
    void ensure_geometry(const sensor_msgs::msg::LaserScan& scan);
    std::optional<double> front_cone_dist(const std::vector<float>& ranges);
    std::optional<WallFit> fit_right_wall(const std::vector<float>& ranges);
    std::optional<EmaState> update_wall(const std::optional<WallFit>& detection);
    void publish_wall(const std::optional<EmaState>& filtered, bool is_right);
    void publish_front(const std::optional<double>& dist);
    void publish_markers(const std::optional<WallFit>& right, const std::optional<WallFit>& front);
    void detection_loop();

    // Parameters
    double min_range_;
    double max_range_;
    double publish_rate_;
    double target_dist_;
    double dist_tol_;
    double angle_tol_;
    double lidar_offset_x_;
    double lidar_offset_y_;
    double right_center_deg_;
    double right_spread_deg_;
    double right_max_fit_dist_;
    double front_cone_half_;
    double front_min_valid_;

    // Scan & state
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    std::string robot_state_{"IDLE"};

    // Cached scan geometry
    std::optional<std::tuple<float, float, size_t>> geom_key_;
    std::vector<double> cos_a_;
    std::vector<double> sin_a_;
    std::vector<double> angles_deg_;
    std::vector<size_t> right_idx_;

    // EMA state
    std::optional<EmaState> right_state_;
    int right_missed_{0};

    // ROS 2 communication handles
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr       state_sub_;

    rclcpp::Publisher<painting_robot_msgs::msg::WallPose>::SharedPtr    right_pub_;
    rclcpp::Publisher<painting_robot_msgs::msg::WallPose>::SharedPtr    front_pub_;
    rclcpp::Publisher<painting_robot_msgs::msg::WallStatus>::SharedPtr  status_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr  marker_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace lidar_wall_detection
