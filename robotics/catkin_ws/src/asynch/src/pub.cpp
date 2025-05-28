#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv) {
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub1 = n.advertise<std_msgs::String>("talker1", 1);
  ros::Publisher chatter_pub2 = n.advertise<std_msgs::String>("talker2", 1);
  ros::Rate loop_rate(3.0);
  std_msgs::String msg1;
  std_msgs::String msg2;

  int count = 0;

  while (ros::ok()) {
    std::stringstream ss1;
    std::stringstream ss2;

    ss1 << "Hey 1:" << count;
    msg1.data = ss1.str();

    ss2 << "Hey 2:" << count;
    msg2.data = ss2.str();

    count++;

    chatter_pub1.publish(msg1);
    chatter_pub2.publish(msg2);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
