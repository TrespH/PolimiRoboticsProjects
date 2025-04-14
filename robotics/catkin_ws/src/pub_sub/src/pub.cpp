#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv) {
	ros::init(argc, argv, "talker"); // (Name must be unique) , ros::init_options::AnonymousName); (random name, not much used)
	ros::NodeHandle n; // Entrypoint for all incoming connections
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1); // We're publishing data
	// [topic name (with '/' we won't have a namespace before it), buffer size (to 1 to discard data)]
	ros::Rate loop_rate(1); // Hz

	while (ros::ok()) {
		std_msgs::String msg;
		msg.data = "hello world!";
		ROS_INFO("%s", msg.data.c_str()); // Good way to debug (also with DEBUG, WARNING, ERROR, etc.)
		chatter_pub.publish(msg);
		ros::spinOnce(); // Call all the ros-related callbacks
		loop_rate.sleep();
	}

	return 0;
}
