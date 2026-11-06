#include <cmath>
#include <limits>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"

class LaserScanBoxFilter : public rclcpp::Node
{
public:

    LaserScanBoxFilter()
        : Node("laser_scan_box_filter")
    {
        // ============================================================
        // FILTER BOX PARAMETERS
        // ============================================================

        // Forward direction
        this->declare_parameter<double>("min_x", -2.5);
        this->declare_parameter<double>("max_x", 2.0);

        // Left / right direction
        this->declare_parameter<double>("min_y", -1.5);
        this->declare_parameter<double>("max_y", -0.2);

        // Laser range limits
        this->declare_parameter<double>("min_range", 0.05);
        this->declare_parameter<double>("max_range", 30.0);

        // Visualization box thickness in Z
        this->declare_parameter<double>("box_z_thickness", 0.02);

        // ============================================================
        // GET PARAMETERS
        // ============================================================

        double p_min_x = this->get_parameter("min_x").as_double();
        double p_max_x = this->get_parameter("max_x").as_double();
        min_x_ = std::min(p_min_x, p_max_x);
        max_x_ = std::max(p_min_x, p_max_x);

        double p_min_y = this->get_parameter("min_y").as_double();
        double p_max_y = this->get_parameter("max_y").as_double();
        min_y_ = std::min(p_min_y, p_max_y);
        max_y_ = std::max(p_min_y, p_max_y);

        min_range_ =
            this->get_parameter("min_range").as_double();

        max_range_ =
            this->get_parameter("max_range").as_double();

        box_z_thickness_ =
            this->get_parameter("box_z_thickness").as_double();

        // ============================================================
        // SUBSCRIBER
        // ============================================================

        scan_sub_ =
            this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/wall_scan",
                rclcpp::SensorDataQoS(),
                std::bind(
                    &LaserScanBoxFilter::scanCallback,
                    this,
                    std::placeholders::_1));

        // ============================================================
        // FILTERED LASER SCAN PUBLISHER
        // ============================================================

        scan_pub_ =
            this->create_publisher<sensor_msgs::msg::LaserScan>(
                "/wall_scan_filtered",
                rclcpp::SensorDataQoS());

        // ============================================================
        // RVIZ BOX MARKER PUBLISHER
        // ============================================================

        marker_pub_ =
            this->create_publisher<visualization_msgs::msg::Marker>(
                "/laser_filter_box",
                10);

        // ============================================================
        // PRINT CONFIGURATION
        // ============================================================

        RCLCPP_INFO(
            this->get_logger(),
            "==========================================");

        RCLCPP_INFO(
            this->get_logger(),
            "LaserScan Box Filter Started");

        RCLCPP_INFO(
            this->get_logger(),
            "Input  : /wall_scan");

        RCLCPP_INFO(
            this->get_logger(),
            "Output : /wall_scan_filtered");

        RCLCPP_INFO(
            this->get_logger(),
            "Marker : /laser_filter_box");

        RCLCPP_INFO(
            this->get_logger(),
            "X: %.2f -> %.2f m",
            min_x_,
            max_x_);

        RCLCPP_INFO(
            this->get_logger(),
            "Y: %.2f -> %.2f m",
            min_y_,
            max_y_);

        RCLCPP_INFO(
            this->get_logger(),
            "Range: %.2f -> %.2f m",
            min_range_,
            max_range_);

        RCLCPP_INFO(
            this->get_logger(),
            "==========================================");
    }


private:

    // ================================================================
    // LASER SCAN CALLBACK
    // ================================================================

    void scanCallback(
        const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // ------------------------------------------------------------
        // Copy original scan
        // ------------------------------------------------------------

        auto filtered_scan = *msg;

        // ------------------------------------------------------------
        // Process every laser measurement
        // ------------------------------------------------------------

        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            const float range = msg->ranges[i];

            // --------------------------------------------------------
            // Invalid measurement
            // --------------------------------------------------------

            if (!std::isfinite(range))
            {
                filtered_scan.ranges[i] =
                    std::numeric_limits<float>::infinity();

                continue;
            }

            // --------------------------------------------------------
            // Range filter
            // --------------------------------------------------------

            if (range < min_range_ ||
                range > max_range_)
            {
                filtered_scan.ranges[i] =
                    std::numeric_limits<float>::infinity();

                continue;
            }

            // --------------------------------------------------------
            // Calculate laser angle
            // --------------------------------------------------------

            const double angle =
                msg->angle_min +
                static_cast<double>(i) *
                msg->angle_increment;

            // --------------------------------------------------------
            // Convert LaserScan -> XY
            // --------------------------------------------------------

            const double x =
                static_cast<double>(range) *
                std::cos(angle);

            const double y =
                static_cast<double>(range) *
                std::sin(angle);

            // --------------------------------------------------------
            // Check whether point is inside box
            // --------------------------------------------------------

            const bool inside_box =
                (x >= min_x_) &&
                (x <= max_x_) &&
                (y >= min_y_) &&
                (y <= max_y_);

            // --------------------------------------------------------
            // Remove points outside box
            // --------------------------------------------------------

            if (!inside_box)
            {
                filtered_scan.ranges[i] =
                    std::numeric_limits<float>::infinity();
            }
        }

        // ------------------------------------------------------------
        // Publish filtered scan
        // ------------------------------------------------------------

        scan_pub_->publish(filtered_scan);

        // ------------------------------------------------------------
        // Publish RViz visualization
        // ------------------------------------------------------------

        publishBoxMarker(msg->header.frame_id);
    }


    // ================================================================
    // RVIZ BOX MARKER
    // ================================================================

    void publishBoxMarker(const std::string & frame_id)
    {
        visualization_msgs::msg::Marker marker;

        // ------------------------------------------------------------
        // Header
        // ------------------------------------------------------------

        marker.header.frame_id = frame_id;
        marker.header.stamp = this->now();

        // ------------------------------------------------------------
        // Marker identification
        // ------------------------------------------------------------

        marker.ns = "laser_scan_filter";
        marker.id = 0;

        // ------------------------------------------------------------
        // Marker type
        // ------------------------------------------------------------

        marker.type =
            visualization_msgs::msg::Marker::CUBE;

        marker.action =
            visualization_msgs::msg::Marker::ADD;

        // ------------------------------------------------------------
        // Position
        //
        // Center of the box
        // ------------------------------------------------------------

        marker.pose.position.x =
            (min_x_ + max_x_) / 2.0;

        marker.pose.position.y =
            (min_y_ + max_y_) / 2.0;

        marker.pose.position.z = 0.0;

        // ------------------------------------------------------------
        // Orientation
        // ------------------------------------------------------------

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // ------------------------------------------------------------
        // Box dimensions
        // ------------------------------------------------------------

        marker.scale.x =
            max_x_ - min_x_;

        marker.scale.y =
            max_y_ - min_y_;

        marker.scale.z =
            box_z_thickness_;

        // ------------------------------------------------------------
        // Marker color
        // ------------------------------------------------------------

        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        // Transparency
        marker.color.a = 0.20;

        // ------------------------------------------------------------
        // Lifetime
        //
        // 0 = marker remains until replaced
        // ------------------------------------------------------------

        marker.lifetime =
            rclcpp::Duration::from_seconds(0.0);

        // ------------------------------------------------------------
        // Publish
        // ------------------------------------------------------------

        marker_pub_->publish(marker);
    }


    // ================================================================
    // PARAMETERS
    // ================================================================

    double min_x_;
    double max_x_;

    double min_y_;
    double max_y_;

    double min_range_;
    double max_range_;

    double box_z_thickness_;

    // ================================================================
    // ROS INTERFACES
    // ================================================================

    rclcpp::Subscription<
        sensor_msgs::msg::LaserScan
    >::SharedPtr scan_sub_;

    rclcpp::Publisher<
        sensor_msgs::msg::LaserScan
    >::SharedPtr scan_pub_;

    rclcpp::Publisher<
        visualization_msgs::msg::Marker
    >::SharedPtr marker_pub_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<LaserScanBoxFilter>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}