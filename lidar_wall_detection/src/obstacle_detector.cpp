#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <algorithm>
#include <string>
#include <functional>
#include <iomanip>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"


class ObstacleDetector : public rclcpp::Node
{
public:

    ObstacleDetector()
        : Node("obstacle_detector")
    {
        // ============================================================
        // DETECTION REGION
        //
        // X = FORWARD / TOP-DOWN DIRECTION
        // Y = LEFT / RIGHT DIRECTION
        // ============================================================

        this->declare_parameter<double>("min_x", -3.0);
        this->declare_parameter<double>("max_x",  1.5);

        this->declare_parameter<double>("min_y", -1.2);
        this->declare_parameter<double>("max_y", -0.20);


        // ============================================================
        // LASER RANGE
        // ============================================================

        this->declare_parameter<double>("min_range", 0.05);
        this->declare_parameter<double>("max_range", 16.0);


        // ============================================================
        // BASELINE
        // ============================================================

        this->declare_parameter<double>("baseline_y", -0.750);
        this->declare_parameter<double>("baseline_offset", 0.000);
        this->declare_parameter<double>("baseline_slope", 0.000);


        // ============================================================
        // NORMAL WALL TOLERANCE
        // ============================================================

        this->declare_parameter<double>(
            "baseline_tolerance",
            0.025);


        // ============================================================
        // OBJECT RETURN THRESHOLD
        // ============================================================

        this->declare_parameter<double>(
            "obstacle_threshold",
            0.05);


        // ============================================================
        // GAP DETECTION
        // ============================================================

        this->declare_parameter<int>(
            "min_gap_points",
            5);

        this->declare_parameter<int>(
            "edge_points",
            2);

        this->declare_parameter<int>(
            "max_index_gap",
            2);


        // ============================================================
        // TEMPORAL FILTER
        // ============================================================

        this->declare_parameter<int>(
            "confirmation_scans",
            1);

        this->declare_parameter<int>(
            "confirmation_index_tolerance",
            8);


        // ============================================================
        // GET PARAMETERS
        // ============================================================

        min_x_ =
            this->get_parameter("min_x").as_double();

        max_x_ =
            this->get_parameter("max_x").as_double();

        min_y_ =
            this->get_parameter("min_y").as_double();

        max_y_ =
            this->get_parameter("max_y").as_double();


        min_range_ =
            this->get_parameter("min_range").as_double();

        max_range_ =
            this->get_parameter("max_range").as_double();


        baseline_y_ =
            this->get_parameter("baseline_y").as_double();

        baseline_offset_ =
            this->get_parameter("baseline_offset").as_double();

        baseline_slope_ =
            this->get_parameter("baseline_slope").as_double();


        baseline_tolerance_ =
            this->get_parameter("baseline_tolerance").as_double();

        obstacle_threshold_ =
            this->get_parameter("obstacle_threshold").as_double();


        min_gap_points_ =
            this->get_parameter("min_gap_points").as_int();

        edge_points_ =
            this->get_parameter("edge_points").as_int();

        max_index_gap_ =
            this->get_parameter("max_index_gap").as_int();


        confirmation_scans_ =
            this->get_parameter("confirmation_scans").as_int();

        confirmation_index_tolerance_ =
            this->get_parameter(
                "confirmation_index_tolerance").as_int();


        // ============================================================
        // SUBSCRIBER
        // ============================================================

        scan_sub_ =
            this->create_subscription<
                sensor_msgs::msg::LaserScan>(
                "/wall_scan_filtered",
                rclcpp::SensorDataQoS(),

                std::bind(
                    &ObstacleDetector::scanCallback,
                    this,
                    std::placeholders::_1));


        // ============================================================
        // OBSTACLE LASER SCAN
        // ============================================================

        obstacle_scan_pub_ =
            this->create_publisher<
                sensor_msgs::msg::LaserScan>(
                "/obstacle_scan",
                rclcpp::SensorDataQoS());


        // ============================================================
        // DETECTION REGION MARKER
        // ============================================================

        marker_pub_ =
            this->create_publisher<
                visualization_msgs::msg::Marker>(
                "/obstacle_detection_box",
                10);


        // ============================================================
        // BASELINE MARKER
        // ============================================================

        baseline_marker_pub_ =
            this->create_publisher<
                visualization_msgs::msg::Marker>(
                "/baseline_visualization",
                10);


        // ============================================================
        // DETECTED POINTS
        // ============================================================

        obstacle_marker_pub_ =
            this->create_publisher<
                visualization_msgs::msg::Marker>(
                "/obstacle_detection_points",
                10);


        // ============================================================
        // DETECTED OBJECT BOUNDING BOX
        // ============================================================

        detected_box_pub_ =
            this->create_publisher<
                visualization_msgs::msg::Marker>(
                "/detected_object_box",
                10);


        // ============================================================
        // RANGE LABELS
        // ============================================================

        range_label_pub_ =
            this->create_publisher<
                visualization_msgs::msg::Marker>(
                "/xy_range_labels",
                10);


        // ============================================================
        // INFO
        // ============================================================

        RCLCPP_INFO(
            this->get_logger(),
            "==========================================");

        RCLCPP_INFO(
            this->get_logger(),
            "       BASELINE GAP DETECTOR");

        RCLCPP_INFO(
            this->get_logger(),
            "==========================================");

        RCLCPP_INFO(
            this->get_logger(),
            "Input             : /wall_scan_filtered");

        RCLCPP_INFO(
            this->get_logger(),
            "Output            : /obstacle_scan");

        RCLCPP_INFO(
            this->get_logger(),
            "X range           : %.3f -> %.3f m",
            min_x_,
            max_x_);

        RCLCPP_INFO(
            this->get_logger(),
            "Y range           : %.3f -> %.3f m",
            min_y_,
            max_y_);

        RCLCPP_INFO(
            this->get_logger(),
            "Baseline Y        : %.3f m",
            baseline_y_);

        RCLCPP_INFO(
            this->get_logger(),
            "Baseline tolerance: %.3f m",
            baseline_tolerance_);

        RCLCPP_INFO(
            this->get_logger(),
            "Obstacle threshold: %.3f m",
            obstacle_threshold_);

        RCLCPP_INFO(
            this->get_logger(),
            "==========================================");
    }


private:

    // ================================================================
    // POINT
    // ================================================================

    struct Point
    {
        size_t index;

        double angle;
        double range;

        double x;
        double y;

        double baseline_range;
        double baseline_y;

        double difference;

        bool missing;
    };


    // ================================================================
    // GAP
    // ================================================================

    struct Gap
    {
        size_t start_index;
        size_t end_index;

        std::vector<Point> points;

        double start_angle;
        double end_angle;

        double width_angle;
        double width_distance;

        double center_x;
        double center_y;
    };


    // ================================================================
    // CALLBACK
    // ================================================================

    void scanCallback(
        const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (msg->ranges.empty())
        {
            return;
        }


        // ============================================================
        // RVIZ VISUALIZATION
        // ============================================================

        publishDetectionRegion(
            msg->header.frame_id);

        publishRangeLabels(
            msg->header.frame_id);

        publishBaseline(
            *msg);


        // ============================================================
        // DETECTION
        // ============================================================

        detectObstacle(*msg);
    }


    // ================================================================
    // BASELINE Y
    // ================================================================

    double getBaselineY(
        double x) const
    {
        return
            baseline_y_ +
            baseline_offset_ +
            baseline_slope_ * x;
    }


    // ================================================================
    // BASELINE RANGE
    //
    // Laser coordinate:
    //
    // x = range * cos(angle)
    // y = range * sin(angle)
    //
    // We solve for intersection with:
    //
    // y = baseline_y + slope*x
    // ================================================================

    bool calculateBaselineRange(
        double angle,
        double &baseline_range) const
    {
        double base_y =
            baseline_y_ +
            baseline_offset_;


        double denominator =
            std::sin(angle) -
            baseline_slope_ *
            std::cos(angle);


        if (std::abs(denominator) < 1e-6)
        {
            return false;
        }


        baseline_range =
            base_y /
            denominator;


        if (!std::isfinite(
                baseline_range))
        {
            return false;
        }


        if (baseline_range <= 0.0)
        {
            return false;
        }


        if (baseline_range < min_range_ ||
            baseline_range > max_range_)
        {
            return false;
        }


        return true;
    }


    // ================================================================
    // BASELINE POINT
    // ================================================================

    bool getBaselinePoint(
        const sensor_msgs::msg::LaserScan &msg,
        size_t index,
        Point &point) const
    {
        if (index >= msg.ranges.size())
        {
            return false;
        }


        double angle =
            msg.angle_min +
            static_cast<double>(index) *
            msg.angle_increment;


        double baseline_range;


        if (!calculateBaselineRange(
                angle,
                baseline_range))
        {
            return false;
        }


        double x =
            baseline_range *
            std::cos(angle);


        double y =
            baseline_range *
            std::sin(angle);


        // ============================================================
        // DETECTION REGION
        //
        // X = forward/top-down
        // Y = left/right
        // ============================================================

        if (x < min_x_ ||
            x > max_x_)
        {
            return false;
        }


        if (y < min_y_ ||
            y > max_y_)
        {
            return false;
        }


        point.index =
            index;

        point.angle =
            angle;

        point.range =
            baseline_range;

        point.x =
            x;

        point.y =
            y;

        point.baseline_range =
            baseline_range;

        point.baseline_y =
            getBaselineY(x);

        point.difference =
            0.0;

        point.missing =
            true;


        return true;
    }


    // ================================================================
    // NORMAL WALL POINT
    // ================================================================

    bool isNormalWallPoint(
        const sensor_msgs::msg::LaserScan &msg,
        size_t index) const
    {
        if (index >= msg.ranges.size())
        {
            return false;
        }


        double current_range =
            static_cast<double>(
                msg.ranges[index]);


        if (!std::isfinite(
                current_range))
        {
            return false;
        }


        if (current_range < min_range_ ||
            current_range > max_range_)
        {
            return false;
        }


        Point baseline;


        if (!getBaselinePoint(
                msg,
                index,
                baseline))
        {
            return false;
        }


        double angle =
            baseline.angle;


        double current_x =
            current_range *
            std::cos(angle);


        double current_y =
            current_range *
            std::sin(angle);


        // ============================================================
        // EXPECTED Y
        // ============================================================

        double expected_y =
            getBaselineY(current_x);


        double y_difference =
            std::abs(
                current_y -
                expected_y);


        if (y_difference >
            baseline_tolerance_)
        {
            return false;
        }


        // ============================================================
        // RANGE DIFFERENCE
        // ============================================================

        double range_difference =
            std::abs(
                current_range -
                baseline.range);


        double range_tolerance =
            std::max(
                0.03,
                baseline_tolerance_);


        if (range_difference >
            range_tolerance)
        {
            return false;
        }


        return true;
    }


    // ================================================================
    // DETECTION
    // ================================================================

    void detectObstacle(
        const sensor_msgs::msg::LaserScan &msg)
    {
        std::vector<bool>
            expected_baseline(
                msg.ranges.size(),
                false);


        std::vector<bool>
            missing(
                msg.ranges.size(),
                false);


        std::vector<bool>
            closer_object(
                msg.ranges.size(),
                false);


        std::vector<Point>
            closer_points;


        // ============================================================
        // CHECK EVERY LASER RAY
        // ============================================================

        for (size_t i = 0;
             i < msg.ranges.size();
             ++i)
        {
            Point baseline;


            if (!getBaselinePoint(
                    msg,
                    i,
                    baseline))
            {
                continue;
            }


            expected_baseline[i] =
                true;


            double current_range =
                static_cast<double>(
                    msg.ranges[i]);


            // ========================================================
            // MISSING POINT
            // ========================================================

            if (!std::isfinite(
                    current_range) ||
                current_range < min_range_ ||
                current_range > max_range_)
            {
                missing[i] =
                    true;

                continue;
            }


            // ========================================================
            // CURRENT XY
            // ========================================================

            double current_x =
                current_range *
                std::cos(baseline.angle);


            double current_y =
                current_range *
                std::sin(baseline.angle);


            // ========================================================
            // RANGE DIFFERENCE
            // ========================================================

            double range_difference =
                baseline.range -
                current_range;


            // ========================================================
            // EXPECTED BASELINE Y
            // ========================================================

            double expected_y =
                getBaselineY(current_x);


            double y_difference =
                std::abs(
                    current_y -
                    expected_y);


            // ========================================================
            // NORMAL WALL
            // ========================================================

            if (y_difference <=
                    baseline_tolerance_ &&
                std::abs(
                    range_difference) <=
                    std::max(
                        0.03,
                        baseline_tolerance_))
            {
                continue;
            }


            // ========================================================
            // OBJECT IN FRONT OF BASELINE
            // ========================================================

            if (range_difference >
                obstacle_threshold_)
            {
                closer_object[i] =
                    true;


                Point p;

                p.index =
                    i;

                p.angle =
                    baseline.angle;

                p.range =
                    current_range;

                p.x =
                    current_x;

                p.y =
                    current_y;

                p.baseline_range =
                    baseline.range;

                p.baseline_y =
                    expected_y;

                p.difference =
                    range_difference;

                p.missing =
                    false;


                closer_points.push_back(p);
            }
        }


        // ============================================================
        // FIND CONTINUOUS MISSING GAPS
        // ============================================================

        std::vector<Gap>
            gaps;


        bool inside_gap =
            false;


        size_t gap_start =
            0;

        size_t gap_end =
            0;


        for (size_t i = 0;
             i < msg.ranges.size();
             ++i)
        {
            if (!expected_baseline[i])
            {
                if (inside_gap)
                {
                    finalizeGap(
                        msg,
                        gap_start,
                        gap_end,
                        gaps);

                    inside_gap =
                        false;
                }

                continue;
            }


            // ========================================================
            // MISSING
            // ========================================================

            if (missing[i])
            {
                if (!inside_gap)
                {
                    gap_start =
                        i;

                    gap_end =
                        i;

                    inside_gap =
                        true;
                }
                else
                {
                    size_t gap =
                        i -
                        gap_end;


                    if (gap <=
                        static_cast<size_t>(
                            max_index_gap_))
                    {
                        gap_end =
                            i;
                    }
                    else
                    {
                        finalizeGap(
                            msg,
                            gap_start,
                            gap_end,
                            gaps);


                        gap_start =
                            i;

                        gap_end =
                            i;
                    }
                }

                continue;
            }


            // ========================================================
            // VALID POINT
            // ========================================================

            if (inside_gap)
            {
                finalizeGap(
                    msg,
                    gap_start,
                    gap_end,
                    gaps);

                inside_gap =
                    false;
            }
        }


        // ============================================================
        // FINAL GAP
        // ============================================================

        if (inside_gap)
        {
            finalizeGap(
                msg,
                gap_start,
                gap_end,
                gaps);
        }


        // ============================================================
        // VALIDATE GAPS
        // ============================================================

        std::vector<Gap>
            valid_gaps;


        for (auto &gap :
             gaps)
        {
            if (gap.points.size() <
                static_cast<size_t>(
                    min_gap_points_))
            {
                continue;
            }


            bool valid_before =
                hasValidWallEdge(
                    msg,
                    gap.start_index,
                    -1);


            bool valid_after =
                hasValidWallEdge(
                    msg,
                    gap.end_index,
                    +1);


            if (!valid_before ||
                !valid_after)
            {
                continue;
            }


            valid_gaps.push_back(
                gap);
        }


        // ============================================================
        // COMBINE DETECTIONS
        // ============================================================

        std::vector<Point>
            final_points;


        // ------------------------------------------------------------
        // GAPS
        // ------------------------------------------------------------

        for (const auto &gap :
             valid_gaps)
        {
            for (const auto &p :
                 gap.points)
            {
                final_points.push_back(
                    p);
            }
        }


        // ------------------------------------------------------------
        // CLOSER OBJECT POINTS
        // ------------------------------------------------------------

        for (const auto &p :
             closer_points)
        {
            final_points.push_back(
                p);
        }


        // ============================================================
        // SORT
        // ============================================================

        std::sort(
            final_points.begin(),
            final_points.end(),

            [](const Point &a,
               const Point &b)
            {
                return a.index <
                       b.index;
            });


        // ============================================================
        // REMOVE DUPLICATES
        // ============================================================

        final_points.erase(
            std::unique(
                final_points.begin(),
                final_points.end(),

                [](const Point &a,
                   const Point &b)
                {
                    return a.index ==
                           b.index;
                }),

            final_points.end());


        // ============================================================
        // NO DETECTION
        // ============================================================

        if (final_points.empty())
        {
            publishEmptyObstacleScan(
                msg);

            publishObstacleMarker(
                msg.header.frame_id,
                final_points);

            publishDetectedObjectBox(
                msg.header.frame_id,
                final_points);

            return;
        }


        // ============================================================
        // OUTPUT SCAN
        // ============================================================

        auto output =
            msg;


        for (auto &r :
             output.ranges)
        {
            r =
                std::numeric_limits<float>
                ::infinity();
        }


        // ============================================================
        // BOUNDARY
        // ============================================================

        double min_object_x =
            std::numeric_limits<double>::max();

        double max_object_x =
            std::numeric_limits<double>::lowest();

        double min_object_y =
            std::numeric_limits<double>::max();

        double max_object_y =
            std::numeric_limits<double>::lowest();


        double maximum_deviation =
            0.0;


        // ============================================================
        // PUT DETECTED POINTS INTO OUTPUT
        // ============================================================

        for (const auto &p :
             final_points)
        {
            output.ranges[p.index] =
                static_cast<float>(
                    p.range);


            min_object_x =
                std::min(
                    min_object_x,
                    p.x);


            max_object_x =
                std::max(
                    max_object_x,
                    p.x);


            min_object_y =
                std::min(
                    min_object_y,
                    p.y);


            max_object_y =
                std::max(
                    max_object_y,
                    p.y);


            maximum_deviation =
                std::max(
                    maximum_deviation,
                    p.difference);
        }


        // ============================================================
        // PUBLISH
        // ============================================================

        obstacle_scan_pub_->publish(
            output);


        publishObstacleMarker(
            msg.header.frame_id,
            final_points);


        publishDetectedObjectBox(
            msg.header.frame_id,
            final_points);


        // ============================================================
        // RESULT
        // ============================================================

        RCLCPP_INFO(
            this->get_logger(),
            "==========================================");

        RCLCPP_INFO(
            this->get_logger(),
            "          OBSTACLE / GAP DETECTED");

        RCLCPP_INFO(
            this->get_logger(),
            "==========================================");


        RCLCPP_INFO(
            this->get_logger(),
            "Valid gaps       : %zu",
            valid_gaps.size());


        RCLCPP_INFO(
            this->get_logger(),
            "Detected points  : %zu",
            final_points.size());


        RCLCPP_INFO(
            this->get_logger(),
            "X range          : %.3f -> %.3f m",
            min_object_x,
            max_object_x);


        RCLCPP_INFO(
            this->get_logger(),
            "Y range          : %.3f -> %.3f m",
            min_object_y,
            max_object_y);


        RCLCPP_INFO(
            this->get_logger(),
            "Maximum deviation: %.3f m",
            maximum_deviation);


        // ============================================================
        // GAP INFORMATION
        // ============================================================

        for (size_t i = 0;
             i < valid_gaps.size();
             ++i)
        {
            const auto &gap =
                valid_gaps[i];


            RCLCPP_INFO(
                this->get_logger(),
                "Gap %zu: index %zu -> %zu",
                i + 1,
                gap.start_index,
                gap.end_index);


            RCLCPP_INFO(
                this->get_logger(),
                "Gap %zu: points=%zu "
                "angle=%.2f -> %.2f deg "
                "width=%.2f deg",
                i + 1,
                gap.points.size(),
                gap.start_angle *
                    180.0 / M_PI,
                gap.end_angle *
                    180.0 / M_PI,
                gap.width_angle *
                    180.0 / M_PI);


            RCLCPP_INFO(
                this->get_logger(),
                "Gap %zu: width=%.3f m "
                "center=(%.3f, %.3f)",
                i + 1,
                gap.width_distance,
                gap.center_x,
                gap.center_y);
        }


        RCLCPP_INFO(
            this->get_logger(),
            "==========================================");
    }


    // ================================================================
    // FINALIZE GAP
    // ================================================================

    void finalizeGap(
        const sensor_msgs::msg::LaserScan &msg,
        size_t start,
        size_t end,
        std::vector<Gap> &gaps)
    {
        if (end < start)
        {
            return;
        }


        size_t count =
            end -
            start +
            1;


        if (count <
            static_cast<size_t>(
                min_gap_points_))
        {
            return;
        }


        Gap gap;


        gap.start_index =
            start;

        gap.end_index =
            end;


        gap.start_angle =
            msg.angle_min +
            static_cast<double>(start) *
            msg.angle_increment;


        gap.end_angle =
            msg.angle_min +
            static_cast<double>(end) *
            msg.angle_increment;


        gap.width_angle =
            std::abs(
                gap.end_angle -
                gap.start_angle);


        gap.center_x =
            0.0;

        gap.center_y =
            0.0;


        // ============================================================
        // GENERATE EXPECTED BASELINE POINTS
        // ============================================================

        for (size_t i = start;
             i <= end;
             ++i)
        {
            Point p;


            if (!getBaselinePoint(
                    msg,
                    i,
                    p))
            {
                continue;
            }


            p.missing =
                true;

            p.difference =
                0.0;


            gap.points.push_back(
                p);


            gap.center_x +=
                p.x;

            gap.center_y +=
                p.y;
        }


        if (gap.points.empty())
        {
            return;
        }


        gap.center_x /=
            static_cast<double>(
                gap.points.size());


        gap.center_y /=
            static_cast<double>(
                gap.points.size());


        // ============================================================
        // PHYSICAL WIDTH
        // ============================================================

        const Point &first =
            gap.points.front();

        const Point &last =
            gap.points.back();


        gap.width_distance =
            std::sqrt(
                std::pow(
                    last.x -
                    first.x,
                    2.0) +

                std::pow(
                    last.y -
                    first.y,
                    2.0));


        gaps.push_back(
            gap);
    }


    // ================================================================
    // VALID WALL EDGE
    // ================================================================

    bool hasValidWallEdge(
        const sensor_msgs::msg::LaserScan &msg,
        size_t gap_index,
        int direction) const
    {
        int found =
            0;


        for (int k = 1;
             k <= edge_points_;
             ++k)
        {
            long index =
                static_cast<long>(
                    gap_index) +
                static_cast<long>(
                    direction) *
                k;


            if (index < 0 ||
                index >=
                    static_cast<long>(
                        msg.ranges.size()))
            {
                return false;
            }


            if (isNormalWallPoint(
                    msg,
                    static_cast<size_t>(
                        index)))
            {
                found++;
            }
        }


        return
            found >=
            edge_points_;
    }


    // ================================================================
    // EMPTY OBSTACLE SCAN
    // ================================================================

    void publishEmptyObstacleScan(
        const sensor_msgs::msg::LaserScan &msg)
    {
        auto output =
            msg;


        for (auto &r :
             output.ranges)
        {
            r =
                std::numeric_limits<float>
                ::infinity();
        }


        obstacle_scan_pub_->publish(
            output);
    }


    // ================================================================
    // BASELINE RVIZ
    // ================================================================

    void publishBaseline(
        const sensor_msgs::msg::LaserScan &msg)
    {
        visualization_msgs::msg::Marker marker;


        marker.header =
            msg.header;


        marker.ns =
            "baseline";


        marker.id =
            0;


        marker.type =
            visualization_msgs::msg::Marker::LINE_STRIP;


        marker.action =
            visualization_msgs::msg::Marker::ADD;


        marker.pose.orientation.w =
            1.0;


        marker.scale.x =
            0.015;


        // CYAN

        marker.color.r =
            0.0;

        marker.color.g =
            1.0;

        marker.color.b =
            1.0;

        marker.color.a =
            1.0;


        // ============================================================
        // BASELINE
        // ============================================================

        for (size_t i = 0;
             i < msg.ranges.size();
             ++i)
        {
            Point p;


            if (!getBaselinePoint(
                    msg,
                    i,
                    p))
            {
                continue;
            }


            geometry_msgs::msg::Point point;


            point.x =
                p.x;

            point.y =
                p.y;

            point.z =
                0.03;


            marker.points.push_back(
                point);
        }


        baseline_marker_pub_->publish(
            marker);
    }


    // ================================================================
    // DETECTION REGION
    //
    // X = FORWARD
    // Y = LEFT / RIGHT
    // ================================================================

    void publishDetectionRegion(
        const std::string &frame_id)
    {
        visualization_msgs::msg::Marker marker;


        marker.header.frame_id =
            frame_id;

        marker.header.stamp =
            this->now();


        marker.ns =
            "detection_region";


        marker.id =
            0;


        marker.type =
            visualization_msgs::msg::Marker::LINE_LIST;


        marker.action =
            visualization_msgs::msg::Marker::ADD;


        marker.pose.orientation.w =
            1.0;


        // ============================================================
        // LINE WIDTH
        // ============================================================

        marker.scale.x =
            0.025;


        // GREEN

        marker.color.r =
            0.0;

        marker.color.g =
            1.0;

        marker.color.b =
            0.0;

        marker.color.a =
            1.0;


        // ============================================================
        // CORNERS
        // ============================================================

        geometry_msgs::msg::Point p1;
        geometry_msgs::msg::Point p2;
        geometry_msgs::msg::Point p3;
        geometry_msgs::msg::Point p4;


        // X MIN / Y MIN

        p1.x =
            min_x_;

        p1.y =
            min_y_;

        p1.z =
            0.02;


        // X MAX / Y MIN

        p2.x =
            max_x_;

        p2.y =
            min_y_;

        p2.z =
            0.02;


        // X MAX / Y MAX

        p3.x =
            max_x_;

        p3.y =
            max_y_;

        p3.z =
            0.02;


        // X MIN / Y MAX

        p4.x =
            min_x_;

        p4.y =
            max_y_;

        p4.z =
            0.02;


        // ============================================================
        // RECTANGLE
        // ============================================================

        marker.points.push_back(p1);
        marker.points.push_back(p2);

        marker.points.push_back(p2);
        marker.points.push_back(p3);

        marker.points.push_back(p3);
        marker.points.push_back(p4);

        marker.points.push_back(p4);
        marker.points.push_back(p1);


        marker_pub_->publish(
            marker);
    }


    // ================================================================
    // RANGE LABELS
    // ================================================================

    void publishRangeLabels(
        const std::string &frame_id)
    {
        // ============================================================
        // X RANGE
        // ============================================================

        visualization_msgs::msg::Marker x_marker;


        x_marker.header.frame_id =
            frame_id;

        x_marker.header.stamp =
            this->now();


        x_marker.ns =
            "range_labels";


        x_marker.id =
            1;


        x_marker.type =
            visualization_msgs::msg::Marker::TEXT_VIEW_FACING;


        x_marker.action =
            visualization_msgs::msg::Marker::ADD;


        x_marker.pose.orientation.w =
            1.0;


        x_marker.pose.position.x =
            (min_x_ + max_x_) / 2.0;

        x_marker.pose.position.y =
            min_y_ - 0.10;

        x_marker.pose.position.z =
            0.10;


        x_marker.scale.z =
            0.10;


        x_marker.color.r =
            1.0;

        x_marker.color.g =
            1.0;

        x_marker.color.b =
            1.0;

        x_marker.color.a =
            1.0;


        std::stringstream x_text;

        x_text
            << "X: "
            << std::fixed
            << std::setprecision(2)
            << min_x_
            << " to "
            << max_x_
            << " m";


        x_marker.text =
            x_text.str();


        range_label_pub_->publish(
            x_marker);


        // ============================================================
        // Y RANGE
        // ============================================================

        visualization_msgs::msg::Marker y_marker;


        y_marker.header.frame_id =
            frame_id;

        y_marker.header.stamp =
            this->now();


        y_marker.ns =
            "range_labels";


        y_marker.id =
            2;


        y_marker.type =
            visualization_msgs::msg::Marker::TEXT_VIEW_FACING;


        y_marker.action =
            visualization_msgs::msg::Marker::ADD;


        y_marker.pose.orientation.w =
            1.0;


        y_marker.pose.position.x =
            min_x_ - 0.15;

        y_marker.pose.position.y =
            (min_y_ + max_y_) / 2.0;

        y_marker.pose.position.z =
            0.10;


        y_marker.scale.z =
            0.10;


        y_marker.color.r =
            1.0;

        y_marker.color.g =
            1.0;

        y_marker.color.b =
            1.0;

        y_marker.color.a =
            1.0;


        std::stringstream y_text;


        y_text
            << "Y: "
            << std::fixed
            << std::setprecision(2)
            << min_y_
            << " to "
            << max_y_
            << " m";


        y_marker.text =
            y_text.str();


        range_label_pub_->publish(
            y_marker);
    }


    // ================================================================
    // OBSTACLE POINT MARKER
    // ================================================================

    void publishObstacleMarker(
        const std::string &frame_id,
        const std::vector<Point> &points)
    {
        visualization_msgs::msg::Marker marker;


        marker.header.frame_id =
            frame_id;

        marker.header.stamp =
            this->now();


        marker.ns =
            "obstacle_points";


        marker.id =
            0;


        // ============================================================
        // CLEAR OLD MARKER
        // ============================================================

        if (points.empty())
        {
            marker.action =
                visualization_msgs::msg::Marker::DELETE;


            obstacle_marker_pub_->publish(
                marker);


            return;
        }


        // ============================================================
        // POINTS
        // ============================================================

        marker.type =
            visualization_msgs::msg::Marker::POINTS;


        marker.action =
            visualization_msgs::msg::Marker::ADD;


        marker.pose.orientation.w =
            1.0;


        marker.scale.x =
            0.05;

        marker.scale.y =
            0.05;


        // RED

        marker.color.r =
            1.0;

        marker.color.g =
            0.0;

        marker.color.b =
            0.0;

        marker.color.a =
            1.0;


        for (const auto &p :
             points)
        {
            geometry_msgs::msg::Point point;


            point.x =
                p.x;

            point.y =
                p.y;

            point.z =
                0.06;


            marker.points.push_back(
                point);
        }


        obstacle_marker_pub_->publish(
            marker);
    }


    // ================================================================
    // DETECTED OBJECT BOUNDING BOX
    // ================================================================

    void publishDetectedObjectBox(
        const std::string &frame_id,
        const std::vector<Point> &points)
    {
        visualization_msgs::msg::Marker marker;


        marker.header.frame_id =
            frame_id;

        marker.header.stamp =
            this->now();


        marker.ns =
            "detected_object_box";


        marker.id =
            0;


        // ============================================================
        // CLEAR
        // ============================================================

        if (points.empty())
        {
            marker.action =
                visualization_msgs::msg::Marker::DELETE;


            detected_box_pub_->publish(
                marker);


            return;
        }


        // ============================================================
        // CALCULATE X/Y RANGE
        // ============================================================

        double min_x =
            std::numeric_limits<double>::max();

        double max_x =
            std::numeric_limits<double>::lowest();

        double min_y =
            std::numeric_limits<double>::max();

        double max_y =
            std::numeric_limits<double>::lowest();


        for (const auto &p :
             points)
        {
            min_x =
                std::min(
                    min_x,
                    p.x);

            max_x =
                std::max(
                    max_x,
                    p.x);

            min_y =
                std::min(
                    min_y,
                    p.y);

            max_y =
                std::max(
                    max_y,
                    p.y);
        }


        // ============================================================
        // RECTANGLE
        // ============================================================

        marker.type =
            visualization_msgs::msg::Marker::LINE_LIST;


        marker.action =
            visualization_msgs::msg::Marker::ADD;


        marker.pose.orientation.w =
            1.0;


        marker.scale.x =
            0.035;


        // MAGENTA / RED

        marker.color.r =
            1.0;

        marker.color.g =
            0.0;

        marker.color.b =
            1.0;

        marker.color.a =
            1.0;


        geometry_msgs::msg::Point p1;
        geometry_msgs::msg::Point p2;
        geometry_msgs::msg::Point p3;
        geometry_msgs::msg::Point p4;


        p1.x =
            min_x;

        p1.y =
            min_y;

        p1.z =
            0.08;


        p2.x =
            max_x;

        p2.y =
            min_y;

        p2.z =
            0.08;


        p3.x =
            max_x;

        p3.y =
            max_y;

        p3.z =
            0.08;


        p4.x =
            min_x;

        p4.y =
            max_y;

        p4.z =
            0.08;


        marker.points.push_back(p1);
        marker.points.push_back(p2);

        marker.points.push_back(p2);
        marker.points.push_back(p3);

        marker.points.push_back(p3);
        marker.points.push_back(p4);

        marker.points.push_back(p4);
        marker.points.push_back(p1);


        detected_box_pub_->publish(
            marker);
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


    double baseline_y_;
    double baseline_offset_;
    double baseline_slope_;


    double baseline_tolerance_;

    double obstacle_threshold_;


    int min_gap_points_;

    int edge_points_;

    int max_index_gap_;


    int confirmation_scans_;

    int confirmation_index_tolerance_;


    // ================================================================
    // ROS
    // ================================================================

    rclcpp::Subscription<
        sensor_msgs::msg::LaserScan
    >::SharedPtr scan_sub_;


    rclcpp::Publisher<
        sensor_msgs::msg::LaserScan
    >::SharedPtr obstacle_scan_pub_;


    rclcpp::Publisher<
        visualization_msgs::msg::Marker
    >::SharedPtr marker_pub_;


    rclcpp::Publisher<
        visualization_msgs::msg::Marker
    >::SharedPtr baseline_marker_pub_;


    rclcpp::Publisher<
        visualization_msgs::msg::Marker
    >::SharedPtr obstacle_marker_pub_;


    rclcpp::Publisher<
        visualization_msgs::msg::Marker
    >::SharedPtr detected_box_pub_;


    rclcpp::Publisher<
        visualization_msgs::msg::Marker
    >::SharedPtr range_label_pub_;
};


// ====================================================================
// MAIN
// ====================================================================

int main(
    int argc,
    char *argv[])
{
    rclcpp::init(
        argc,
        argv);


    auto node =
        std::make_shared<
            ObstacleDetector
        >();


    rclcpp::spin(
        node);


    rclcpp::shutdown();


    return 0;
}