#include "lidar_wall_detection/wall_detection_node.hpp"

namespace lidar_wall_detection {

// ── RANSAC Line Fit ───────────────────────────────────────────────────────────
std::optional<RansacResult> ransac_line(
    const std::vector<Eigen::Vector2d>& points,
    int n_iter,
    double threshold,
    int min_inliers)
{
    const size_t n_pts = points.size();
    if (n_pts < 2) return std::nullopt;

    static thread_local std::mt19937 rng(42);
    std::uniform_int_distribution<size_t> dist(0, n_pts - 1);

    RansacResult best;
    best.inlier_count = 0;

    for (int iter = 0; iter < n_iter; ++iter) {
        size_t idx1 = dist(rng);
        size_t idx2 = dist(rng);
        while (idx2 == idx1) {
            idx2 = dist(rng);
        }

        const auto& p1 = points[idx1];
        const auto& p2 = points[idx2];

        Eigen::Vector2d direction = p2 - p1;
        double seg_len = direction.norm();
        if (seg_len < 1e-6) continue;

        Eigen::Vector2d n(-direction.y() / seg_len, direction.x() / seg_len);
        double d = n.dot(p1);

        if (d < 0.0) {
            n = -n;
            d = -d;
        }

        std::vector<bool> mask(n_pts, false);
        int count = 0;
        for (size_t i = 0; i < n_pts; ++i) {
            double dist_to_line = std::abs(points[i].dot(n) - d);
            if (dist_to_line < threshold) {
                mask[i] = true;
                count++;
            }
        }

        if (count > best.inlier_count) {
            best.inlier_count = count;
            best.inlier_mask  = std::move(mask);
            best.normal       = n;
            best.d            = d;
        }
    }

    if (best.inlier_count < min_inliers) {
        return std::nullopt;
    }

    return best;
}

// ── PCA Line Fit ──────────────────────────────────────────────────────────────
std::optional<PcaResult> pca_fit(const std::vector<Eigen::Vector2d>& arc_points) {
    if (arc_points.size() < 5) {
        return std::nullopt;
    }

    Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
    for (const auto& p : arc_points) {
        centroid += p;
    }
    centroid /= static_cast<double>(arc_points.size());

    Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
    for (const auto& p : arc_points) {
        Eigen::Vector2d centered = p - centroid;
        cov += centered * centered.transpose();
    }
    cov /= static_cast<double>(arc_points.size());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(cov);
    if (solver.info() != Eigen::Success) {
        return std::nullopt;
    }

    Eigen::Vector2d eigenvalues = solver.eigenvalues(); // sorted ascending: [0]=min, [1]=max
    Eigen::Vector2d normal = solver.eigenvectors().col(0); // smallest eigenvector = normal

    // Flip normal to point toward robot at origin
    if (normal.dot(centroid) > 0.0) {
        normal = -normal;
    }

    double distance = std::abs(normal.dot(centroid));
    double ratio = eigenvalues(1) / (eigenvalues(0) + 1e-9);

    return PcaResult{normal, distance, centroid, eigenvalues, ratio};
}

// ── Range Check ───────────────────────────────────────────────────────────────
RangeCheckResult range_check(
    const std::vector<float>& ranges,
    const std::vector<double>& angles_deg,
    double center_deg,
    double window_deg,
    std::optional<double> max_dist,
    std::optional<double> fitted_dist,
    double rel_tolerance)
{
    std::vector<double> window_ranges;
    for (size_t i = 0; i < ranges.size(); ++i) {
        double diff = std::abs(angle_diff_deg(angles_deg[i], center_deg));
        if (diff <= window_deg) {
            float r = ranges[i];
            if (std::isfinite(r) && r > 0.05f) {
                window_ranges.push_back(static_cast<double>(r));
            }
        }
    }

    if (window_ranges.size() < 3) {
        return {0.0, false, "insufficient rays"};
    }

    size_t mid_idx = window_ranges.size() / 2;
    std::nth_element(window_ranges.begin(), window_ranges.begin() + mid_idx, window_ranges.end());
    double median_r = window_ranges[mid_idx];

    if (max_dist.has_value() && median_r > max_dist.value()) {
        return {median_r, false, "median > max_dist"};
    }

    if (fitted_dist.has_value()) {
        double rel_err = std::abs(median_r - fitted_dist.value()) / (fitted_dist.value() + 1e-6);
        if (rel_err > rel_tolerance) {
            return {median_r, false, "median differs from fitted"};
        }
    }

    return {median_r, true, "ok"};
}

// ── Theta Check ───────────────────────────────────────────────────────────────
ThetaCheckResult theta_check(const PcaResult& pca_result, double target_deg, double tolerance_deg) {
    double theta = std::atan2(pca_result.normal.y(), pca_result.normal.x()) * 180.0 / M_PI;
    double diff = std::abs(angle_diff_deg(theta, target_deg));
    if (diff > tolerance_deg) {
        return {theta, false, "theta diff > tol"};
    }
    return {theta, true, "ok"};
}

// ==============================================================================
// WallDetectionNode Class Implementation
// ==============================================================================
WallDetectionNode::WallDetectionNode(const rclcpp::NodeOptions & options)
    : Node("wall_detection_node", options)
{
    // Parameters
    this->declare_parameter<double>("min_range", MIN_RANGE_DEFAULT);
    this->declare_parameter<double>("max_range", MAX_RANGE_DEFAULT);
    this->declare_parameter<double>("publish_rate", 10.0);
    this->declare_parameter<double>("target_distance", 0.770);
    this->declare_parameter<double>("dist_tolerance", 0.10);
    this->declare_parameter<double>("angle_tolerance", 0.02);
    this->declare_parameter<double>("lidar_offset_x", 0.195);
    this->declare_parameter<double>("lidar_offset_y", -0.147);
    this->declare_parameter<double>("right_center_deg", RIGHT_CENTER_DEF);
    this->declare_parameter<double>("right_spread_deg", RIGHT_SPREAD_DEF);
    this->declare_parameter<double>("right_max_fit_dist", RIGHT_MAX_DIST_DEF);
    this->declare_parameter<double>("front_cone_half_deg", FRONT_CONE_HALF_DEF);
    this->declare_parameter<double>("front_min_valid", FRONT_MIN_VALID_DEF);

    min_range_          = this->get_parameter("min_range").as_double();
    max_range_          = this->get_parameter("max_range").as_double();
    publish_rate_       = this->get_parameter("publish_rate").as_double();
    target_dist_        = this->get_parameter("target_distance").as_double();
    dist_tol_           = this->get_parameter("dist_tolerance").as_double();
    angle_tol_          = this->get_parameter("angle_tolerance").as_double();
    lidar_offset_x_     = this->get_parameter("lidar_offset_x").as_double();
    lidar_offset_y_     = this->get_parameter("lidar_offset_y").as_double();
    right_center_deg_   = this->get_parameter("right_center_deg").as_double();
    right_spread_deg_   = this->get_parameter("right_spread_deg").as_double();
    right_max_fit_dist_ = this->get_parameter("right_max_fit_dist").as_double();
    front_cone_half_    = this->get_parameter("front_cone_half_deg").as_double();
    front_min_valid_    = this->get_parameter("front_min_valid").as_double();

    // Subscriptions
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&WallDetectionNode::scan_callback, this, std::placeholders::_1));

    state_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/robot_state", 10, std::bind(&WallDetectionNode::state_callback, this, std::placeholders::_1));

    // Publishers
    right_pub_  = this->create_publisher<painting_robot_msgs::msg::WallPose>("/right_wall_pose", 10);
    front_pub_  = this->create_publisher<painting_robot_msgs::msg::WallPose>("/front_wall_pose", 10);
    status_pub_ = this->create_publisher<painting_robot_msgs::msg::WallStatus>("/wall_status", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/wall_markers", 10);

    // Periodic detection timer
    auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&WallDetectionNode::detection_loop, this));

    RCLCPP_INFO(this->get_logger(), "WallDetectionNode (C++ lidar_wall_detection) initialized successfully");
}

void WallDetectionNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    latest_scan_ = msg;
}

void WallDetectionNode::state_callback(const std_msgs::msg::String::SharedPtr msg) {
    robot_state_ = msg->data;
}

void WallDetectionNode::ensure_geometry(const sensor_msgs::msg::LaserScan& scan) {
    const size_t n = scan.ranges.size();
    std::tuple<float, float, size_t> key{scan.angle_min, scan.angle_increment, n};

    if (geom_key_ && *geom_key_ == key) {
        return;
    }

    cos_a_.resize(n);
    sin_a_.resize(n);
    angles_deg_.resize(n);
    right_idx_.clear();

    const double center_rad = right_center_deg_ * M_PI / 180.0;
    const double spread_rad = right_spread_deg_ * M_PI / 180.0;

    for (size_t i = 0; i < n; ++i) {
        double angle = std::fmod(scan.angle_min + i * scan.angle_increment + LIDAR_YAW_OFFSET, 2.0 * M_PI);
        if (angle < 0.0) angle += 2.0 * M_PI;

        cos_a_[i] = std::cos(angle);
        sin_a_[i] = std::sin(angle);
        angles_deg_[i] = angle * 180.0 / M_PI;

        double diff = angle_diff(angle, center_rad);
        if (std::abs(diff) <= spread_rad) {
            right_idx_.push_back(i);
        }
    }

    geom_key_ = key;
    RCLCPP_INFO(this->get_logger(), "[wall] C++ geometry cache rebuilt: n=%zu, right_arc_beams=%zu",
                n, right_idx_.size());
}

std::optional<double> WallDetectionNode::front_cone_dist(const std::vector<float>& ranges) {
    std::vector<double> valid_ranges;
    for (size_t i = 0; i < ranges.size(); ++i) {
        double deg = angles_deg_[i];
        bool in_cone = (deg <= front_cone_half_ || deg >= 360.0 - front_cone_half_);
        if (in_cone) {
            float r = ranges[i];
            if (std::isfinite(r) && r > front_min_valid_) {
                valid_ranges.push_back(static_cast<double>(r));
            }
        }
    }

    if (valid_ranges.size() < 3) {
        return std::nullopt;
    }

    size_t mid_idx = valid_ranges.size() / 2;
    std::nth_element(valid_ranges.begin(), valid_ranges.begin() + mid_idx, valid_ranges.end());
    return valid_ranges[mid_idx];
}

std::optional<WallFit> WallDetectionNode::fit_right_wall(const std::vector<float>& ranges) {
    if (right_idx_.empty()) return std::nullopt;

    std::vector<Eigen::Vector2d> right_pts;
    for (size_t idx : right_idx_) {
        if (idx >= ranges.size()) continue;
        float r = ranges[idx];
        if (std::isfinite(r) && r > min_range_ && r < max_range_) {
            double x = static_cast<double>(r) * cos_a_[idx];
            double y = static_cast<double>(r) * sin_a_[idx];
            double pt_dist = std::sqrt(x * x + y * y);
            if (pt_dist < right_max_fit_dist_) {
                right_pts.emplace_back(x, y);
            }
        }
    }

    if (right_pts.size() < 2) return std::nullopt;

    auto ransac_res = ransac_line(right_pts, RANSAC_ITERATIONS, RANSAC_THRESHOLD, RANSAC_MIN_INLIERS);
    if (!ransac_res.has_value()) return std::nullopt;

    std::vector<Eigen::Vector2d> inliers;
    for (size_t i = 0; i < right_pts.size(); ++i) {
        if (ransac_res->inlier_mask[i]) {
            inliers.push_back(right_pts[i]);
        }
    }

    auto pca_res = pca_fit(inliers);
    if (!pca_res.has_value()) return std::nullopt;

    auto r_check = range_check(ranges, angles_deg_, right_center_deg_, 5.0,
                               right_max_fit_dist_, pca_res->distance, RANGE_TOL);
    if (!r_check.passed) return std::nullopt;

    auto t_check = theta_check(*pca_res, RIGHT_TARGET_DEG, THETA_TOL_DEG);
    if (!t_check.passed) return std::nullopt;

    double theta_rad = std::atan2(pca_res->normal.y(), pca_res->normal.x());
    return WallFit{pca_res->ratio, pca_res->distance, theta_rad, pca_res->normal};
}

std::optional<EmaState> WallDetectionNode::update_wall(const std::optional<WallFit>& detection) {
    if (!detection.has_value()) {
        right_missed_++;
        if (right_missed_ > MAX_MISSED_FRAMES) {
            right_state_.reset();
            return std::nullopt;
        }
        return right_state_;
    }

    right_missed_ = 0;
    double d_new = detection->distance;
    double theta_new = detection->theta;

    if (!right_state_.has_value()) {
        right_state_ = EmaState{d_new, theta_new};
    } else {
        double d_prev = right_state_->distance;
        double theta_prev = right_state_->theta;
        double d_ema = EMA_ALPHA * d_new + (1.0 - EMA_ALPHA) * d_prev;
        double theta_ema = theta_prev + EMA_ALPHA * angle_diff(theta_new, theta_prev);
        right_state_ = EmaState{d_ema, theta_ema};
    }

    return right_state_;
}

void WallDetectionNode::publish_wall(const std::optional<EmaState>& filtered, bool is_right) {
    auto stamp = this->now();
    painting_robot_msgs::msg::WallPose msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "laser_frame";

    std::string label = is_right ? "Right" : "Front";

    if (!filtered.has_value()) {
        msg.valid = false;
        if (is_right) {
            right_pub_->publish(msg);
            if (robot_state_ != "CORNER_LOGIC") {
                painting_robot_msgs::msg::WallStatus status;
                status.header.stamp = stamp;
                status.header.frame_id = "base_link";
                status.wall_valid = false;
                status.aligned = false;
                status.distance_error = 0.0f;
                status.distance_error_corrected = 0.0f;
                status.angle_error = 0.0f;
                status_pub_->publish(status);
            }
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "%s wall — NOT DETECTED", label.c_str());
        } else {
            front_pub_->publish(msg);
        }
        return;
    }

    double d = filtered->distance;
    double theta = filtered->theta;
    double target_theta = is_right ? RIGHT_WALL_THETA : FRONT_WALL_THETA;
    double angle_error = angle_diff(theta, target_theta);

    msg.distance = static_cast<float>(d);
    msg.angle = static_cast<float>(angle_error);
    msg.valid = true;

    if (is_right) {
        right_pub_->publish(msg);

        double dist_error = d - target_dist_;
        double dist_error_corrected = dist_error
            + lidar_offset_x_ * std::sin(angle_error)
            + lidar_offset_y_ * (1.0 - std::cos(angle_error));

        bool aligned = (std::abs(dist_error) < dist_tol_) && (std::abs(angle_error) < angle_tol_);

        if (robot_state_ != "CORNER_LOGIC") {
            painting_robot_msgs::msg::WallStatus status;
            status.header.stamp = stamp;
            status.header.frame_id = "base_link";
            status.wall_valid = true;
            status.aligned = aligned;
            status.distance_error = static_cast<float>(dist_error);
            status.distance_error_corrected = static_cast<float>(dist_error_corrected);
            status.angle_error = static_cast<float>(angle_error);
            status_pub_->publish(status);
        }

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "%s Lidar->Wall — dist=%.3fm target=%.3fm dist_err=%+.3fm Center->Wall- dist=%+.3fm angle_err=%.1fdeg aligned=%s",
            label.c_str(), d, target_dist_, dist_error, dist_error_corrected,
            angle_error * 180.0 / M_PI, aligned ? "True" : "False");
    } else {
        front_pub_->publish(msg);
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "%s wall — dist=%.3fm angle_err=%.1fdeg",
            label.c_str(), d, angle_error * 180.0 / M_PI);
    }
}

void WallDetectionNode::publish_front(const std::optional<double>& dist) {
    auto stamp = this->now();
    painting_robot_msgs::msg::WallPose msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "laser_frame";

    if (!dist.has_value()) {
        msg.valid = false;
        front_pub_->publish(msg);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Front wall — NOT DETECTED");
        return;
    }

    msg.distance = static_cast<float>(*dist);
    msg.angle = 0.0f;
    msg.valid = true;
    front_pub_->publish(msg);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Front wall — dist=%.3fm (cone median)", *dist);
}

void WallDetectionNode::publish_markers(const std::optional<WallFit>& right, const std::optional<WallFit>& front) {
    (void)front;
    visualization_msgs::msg::MarkerArray ma;
    auto t = this->now();

    visualization_msgs::msg::Marker delete_all;
    delete_all.header.frame_id = "base_link";
    delete_all.header.stamp = t;
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back(delete_all);

    auto add_wall_marker = [&](const std::optional<WallFit>& candidate, float r, float g, float b, int marker_id) {
        if (!candidate.has_value()) return;

        double d = candidate->distance;
        const auto& n = candidate->normal;
        Eigen::Vector2d dir_vec(-n.y(), n.x());
        Eigen::Vector2d p0 = n * d;
        Eigen::Vector2d p1 = p0 + dir_vec * 3.0;
        Eigen::Vector2d p2 = p0 - dir_vec * 3.0;

        // Fitted line strip
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "base_link";
        m.header.stamp = t;
        m.ns = "walls";
        m.id = marker_id;
        m.type = visualization_msgs::msg::Marker::LINE_STRIP;
        m.scale.x = 0.04;
        m.color.r = r;
        m.color.g = g;
        m.color.b = b;
        m.color.a = 1.0f;

        geometry_msgs::msg::Point pt1, pt2;
        pt1.x = p1.x(); pt1.y = p1.y(); pt1.z = 0.0;
        pt2.x = p2.x(); pt2.y = p2.y(); pt2.z = 0.0;
        m.points.push_back(pt1);
        m.points.push_back(pt2);
        ma.markers.push_back(m);

        // Normal arrow
        visualization_msgs::msg::Marker m2;
        m2.header.frame_id = "base_link";
        m2.header.stamp = t;
        m2.ns = "normals";
        m2.id = marker_id + 100;
        m2.type = visualization_msgs::msg::Marker::ARROW;
        m2.scale.x = 0.06;
        m2.scale.y = 0.12;
        m2.scale.z = 0.12;
        m2.color.r = 1.0f;
        m2.color.g = 1.0f;
        m2.color.b = 0.0f;
        m2.color.a = 1.0f;

        geometry_msgs::msg::Point arrow_start, arrow_end;
        arrow_start.x = p0.x(); arrow_start.y = p0.y(); arrow_start.z = 0.0;
        arrow_end.x = p0.x() + n.x(); arrow_end.y = p0.y() + n.y(); arrow_end.z = 0.0;
        m2.points.push_back(arrow_start);
        m2.points.push_back(arrow_end);
        ma.markers.push_back(m2);
    };

    add_wall_marker(right, 0.0f, 1.0f, 0.0f, 1); // green — right wall
    marker_pub_->publish(ma);
}

void WallDetectionNode::detection_loop() {
    if (!latest_scan_) return;

    ensure_geometry(*latest_scan_);
    const auto& ranges = latest_scan_->ranges;

    // 1. Right wall fit (RANSAC + PCA)
    auto right_result = fit_right_wall(ranges);

    // 2. Right wall EMA filter & publish
    auto right_filtered = update_wall(right_result);
    publish_wall(right_filtered, true);

    // 3. Front wall cone median
    auto front_dist = front_cone_dist(ranges);
    publish_front(front_dist);

    // 4. Markers
    publish_markers(right_result, std::nullopt);
}

} // namespace lidar_wall_detection
