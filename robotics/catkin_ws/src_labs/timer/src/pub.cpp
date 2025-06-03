#include "ros/ros.h"
#include "std_msgs/String.h"
#include <time.h>

void timerCallback(const ros::TimerEvent &ev) {
	ROS_INFO_STREAM("Callback called at time: " << ros::Time::now());
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "timed_talker");
	ros::NodeHandle n;
	ros::Timer timer = n.createTimer(ros::Duration(1), timerCallback);
	ros::spin();
	return 0;
}
