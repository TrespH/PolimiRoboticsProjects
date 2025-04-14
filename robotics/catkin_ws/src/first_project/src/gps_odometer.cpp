#include "ros/ros.h"
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String::ConstPtr &msg) { // make sure it has the same type as the publisher
	ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("chatter", 1, chatterCallback);
	ros::spin(); // Not at a specific frequency, instead as fast as possible
	return 0;
}
