#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <algorithm>
#include <cmath>

#define ROBOT_RADIUS 0.3 // meters, adjust to your platform’s footprint

class LaserScanMerger {
public:
    LaserScanMerger() {
        ros::NodeHandle nh;
        front_sub = nh.subscribe("/scan_front", 1, &LaserScanMerger::frontCallback, this);
        back_sub = nh.subscribe("/scan_back", 1, &LaserScanMerger::backCallback, this);
        merged_pub = nh.advertise<sensor_msgs::LaserScan>("/scan_merged", 1);
		logParameters();
    }
	
	void frontCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        latest_front = msg;
        tryMergeAndPublish();
    }

    void backCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        latest_back = msg;
        tryMergeAndPublish();
    }
	
	// Merge and publish when both scans are updated
    void tryMergeAndPublish() {
        if (!latest_front || !latest_back) return;

        // Check timestamps difference
        ros::Duration dt = latest_front->header.stamp - latest_back->header.stamp;
        if (std::abs(dt.toSec()) > 0.06) // allow up to 30/50?ms drift (observed one is 20ms)
            return;

        // Compose merged scan params
        sensor_msgs::LaserScan merged = *latest_front;

        std::vector<float> merged_ranges;
        std::vector<float> merged_intensities;

		// Front
		for (size_t i = I_MIN; i <= I_MAX; i++) {
			float r = latest_front->ranges[i];
			merged_ranges.push_back((r > ROBOT_RADIUS) ? r : 20); // 20 or std::numeric_limits<float>::infinity());
			//if (!latest_front->intensities.empty())merged_intensities.push_back(latest_front->intensities[i]);
		}
		// Back
		for (size_t i = I_MIN; i <= I_MAX; i++) {
			float r = latest_back->ranges[i];
			merged_ranges.push_back((r > ROBOT_RADIUS) ? r : 20); // 20 or std::numeric_limits<float>::infinity());
			//if (!latest_back->intensities.empty()) merged_intensities.push_back(latest_back->intensities[i]);
		}

		merged.header.stamp = ros::Time::now();
		merged.header.frame_id = "base_link";
		merged.angle_min = MERGED_ANGLE_MIN;
		merged.angle_max = MERGED_ANGLE_MAX;
		merged.angle_increment = ANGLE_INCREMENT;
		merged.ranges = merged_ranges;
		//if (!merged_intensities.empty()) merged.intensities = merged_intensities;
		merged.time_increment = TIME_INCREMENT;
		merged.scan_time = SCAN_TIME;

		merged_pub.publish(merged);
	}

private:
    ros::Subscriber front_sub;
    ros::Subscriber back_sub;
    ros::Publisher merged_pub;

    sensor_msgs::LaserScan::ConstPtr latest_front;
    sensor_msgs::LaserScan::ConstPtr latest_back;
	
	// Angle params (observed they're the same for back and front)
	const double ANGLE_MIN = -2.356194496154785;
	const double ANGLE_MAX = 2.3557233810424805;
	const double ANGLE_INCREMENT = 0.00581718236207962;
	
	const double ANGLE_LEFT_CUT = -M_PI/2; // -90° in rads
	const double ANGLE_RIGHT_CUT =  M_PI/2; // +90° in rads
	
	const int I_MIN = (int)round((ANGLE_LEFT_CUT - ANGLE_MIN) / ANGLE_INCREMENT); // =~ 135 scans
	const int I_MAX = (int)round((ANGLE_RIGHT_CUT - ANGLE_MIN) / ANGLE_INCREMENT); // =~ 674 scans
	// => total considered scans (per laser): 810 - 2 * 135 =~ 540 scans
	
	const double MERGED_ANGLE_MIN = ANGLE_LEFT_CUT; // = -90° in rads
	const double MERGED_ANGLE_MAX = ANGLE_RIGHT_CUT + (ANGLE_RIGHT_CUT - ANGLE_LEFT_CUT); // =~ 270° in rads
	
	const int RANGE_MIN = 0;
	const int RANGE_MAX = 100;
	const double TIME_INCREMENT = 0.00006172839493956417;
	const double SCAN_TIME = 0.06666667014360428;

	void logParameters() const {
		ROS_INFO_STREAM("LaserScanMerger parameters:");
		ROS_INFO_STREAM("  ANGLE_MIN        = " << ANGLE_MIN);
		ROS_INFO_STREAM("  ANGLE_MAX        = " << ANGLE_MAX);
		ROS_INFO_STREAM("  ANGLE_INCREMENT  = " << ANGLE_INCREMENT);
		ROS_INFO_STREAM("  ANGLE_LEFT_CUT   = " << ANGLE_LEFT_CUT);
		ROS_INFO_STREAM("  ANGLE_RIGHT_CUT  = " << ANGLE_RIGHT_CUT);
		ROS_INFO_STREAM("  Number of scans  = " << floor((ANGLE_MAX - ANGLE_MIN) / ANGLE_INCREMENT) + 1);
		ROS_INFO_STREAM("  I_MIN            = " << I_MIN);
		ROS_INFO_STREAM("  I_MAX            = " << I_MAX);
		ROS_INFO_STREAM("  MERGED_ANGLE_MIN = " << MERGED_ANGLE_MIN);
		ROS_INFO_STREAM("  MERGED_ANGLE_MAX = " << MERGED_ANGLE_MAX);
		ROS_INFO_STREAM("  RANGE_MIN        = " << RANGE_MIN);
		ROS_INFO_STREAM("  RANGE_MAX        = " << RANGE_MAX);
		ROS_INFO_STREAM("  TIME_INCREMENT   = " << TIME_INCREMENT);
		ROS_INFO_STREAM("  SCAN_TIME        = " << SCAN_TIME);
		ROS_INFO_STREAM("  ROBOT_RADIUS     = " << ROBOT_RADIUS);
	}
};	

int main(int argc, char** argv) {
    ros::init(argc, argv, "laser_scan_merger");
    LaserScanMerger merger;
    ros::spin();
    return 0;
}