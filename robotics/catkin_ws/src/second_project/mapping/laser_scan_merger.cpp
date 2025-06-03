#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <algorithm>
#include <cmath>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define ROBOT_RADIUS 0.4 // meters to filter from LIDAR scans

class LaserScanMerger {
public:
    LaserScanMerger() : tf_buffer(), tf_listener(tf_buffer) {
        ros::NodeHandle nh;
        back_sub  = nh.subscribe("/scan_back",  10, &LaserScanMerger::backCallback,  this);
        front_sub = nh.subscribe("/scan_front", 10, &LaserScanMerger::frontCallback, this);
        merged_pub = nh.advertise<sensor_msgs::LaserScan>("/scan_merged", 100);
        logParameters();
    }

    void backCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        latest_back = msg;
        tryMergeAndPublish();
    }

    void frontCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        latest_front = msg;
        tryMergeAndPublish();
    }

    void tryMergeAndPublish() {
        if (!latest_back || !latest_front) return;

        ros::Duration dt = latest_front->header.stamp - latest_back->header.stamp;
        if (std::abs(dt.toSec()) > 0.06) return; // Allow up to 60ms drift

        // Prepare merged LaserScan
        sensor_msgs::LaserScan merged;
        merged.header.stamp = ros::Time::now();
        merged.header.frame_id = "base_link";
        merged.angle_min = MERGED_ANGLE_MIN;
        merged.angle_max = MERGED_ANGLE_MAX;
        merged.angle_increment = ANGLE_INCREMENT;
        merged.ranges.resize(NUM_BINS, std::numeric_limits<float>::infinity());
        merged.intensities.resize(NUM_BINS, 0.0);

        processScan(latest_back, merged);
        processScan(latest_front, merged);

        merged_pub.publish(merged);

        latest_back.reset();
        latest_front.reset();
    }

    // Transform each point of a scan into base_link and bin into merged scan
    void processScan(const sensor_msgs::LaserScan::ConstPtr& scan, sensor_msgs::LaserScan& merged) {
        geometry_msgs::PointStamped scan_point, base_point;

        for (int i = 0; i < scan->ranges.size(); i++) {
            float r = scan->ranges[i];
            if (r < ROBOT_RADIUS || std::isnan(r) || !std::isfinite(r)) continue;

            float angle = ANGLE_MIN + i * ANGLE_INCREMENT;
			if (angle < ANGLE_LEFT_CUT || angle > ANGLE_RIGHT_CUT) continue;

            scan_point.header.frame_id = scan->header.frame_id;
            scan_point.header.stamp = scan->header.stamp;
            scan_point.point.x = r * std::cos(angle);
            scan_point.point.y = r * std::sin(angle);
            scan_point.point.z = 0.0;

            try {
                tf_buffer.transform(scan_point, base_point, "base_link", ros::Duration(0.05));
            } catch (tf2::TransformException &ex) {
                ROS_WARN("TF point transform failed: %s", ex.what());
                continue;
            }

            float base_r = std::hypot(base_point.point.x, base_point.point.y);
            float base_angle = std::atan2(base_point.point.y, base_point.point.x);

            int idx = (int)((base_angle - MERGED_ANGLE_MIN) / ANGLE_INCREMENT);
            if (idx >= 0 && idx < merged.ranges.size()) {
                if (base_r < merged.ranges[idx]) {
                    merged.ranges[idx] = std::min(merged.ranges[idx], base_r);
                    merged.intensities[idx] = scan->intensities[i];
                }
            }
        }
    }

private:
    ros::Subscriber front_sub;
    ros::Subscriber back_sub;
    ros::Publisher merged_pub;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    sensor_msgs::LaserScan::ConstPtr latest_front;
    sensor_msgs::LaserScan::ConstPtr latest_back;

    // LaserScan parameters
    const double ANGLE_MIN = -2.356194496154785;
    const double ANGLE_MAX = 2.3557233810424805;
    const double ANGLE_INCREMENT = 0.00581718236207962;
    const double ANGLE_LEFT_CUT = -M_PI / 2; // -90° in radians
    const double ANGLE_RIGHT_CUT = M_PI / 2; // +90° in radians
    const double MERGED_ANGLE_MIN = ANGLE_LEFT_CUT; // -90° in radians
    const double MERGED_ANGLE_MAX = ANGLE_RIGHT_CUT + (ANGLE_RIGHT_CUT - ANGLE_LEFT_CUT); // +270° in radians
	const int NUM_BINS = (int)((MERGED_ANGLE_MAX - MERGED_ANGLE_MIN) / ANGLE_INCREMENT); // 1080
    const double TIME_INCREMENT = 0.00006172839493956417;
    const double SCAN_TIME = 0.06666667014360428;

    void logParameters() const {
        ROS_INFO_STREAM("LaserScanMerger parameters:");
        ROS_INFO_STREAM("  ANGLE_MIN        = " << ANGLE_MIN);
        ROS_INFO_STREAM("  ANGLE_MAX        = " << ANGLE_MAX);
        ROS_INFO_STREAM("  ANGLE_INCREMENT  = " << ANGLE_INCREMENT);
        ROS_INFO_STREAM("  ANGLE_LEFT_CUT   = " << ANGLE_LEFT_CUT);
        ROS_INFO_STREAM("  ANGLE_RIGHT_CUT  = " << ANGLE_RIGHT_CUT);
        ROS_INFO_STREAM("  MERGED_ANGLE_MIN = " << MERGED_ANGLE_MIN);
        ROS_INFO_STREAM("  MERGED_ANGLE_MAX = " << MERGED_ANGLE_MAX);
        ROS_INFO_STREAM("  NUM_BINS		    = " << NUM_BINS);
        ROS_INFO_STREAM("  TIME_INCREMENT   = " << TIME_INCREMENT);
        ROS_INFO_STREAM("  SCAN_TIME        = " << SCAN_TIME);
        ROS_INFO_STREAM("  ROBOT_RADIUS     = " << ROBOT_RADIUS);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "laser_scan_merger");
    LaserScanMerger merger;
    ros::spin();
    return 0;
}