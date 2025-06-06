#include "geometry_msgs/Vector3Stamped.h"
#include "ros/ros.h"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

void callback(const geometry_msgs::Vector3StampedConstPtr &msg1,
              const geometry_msgs::Vector3StampedConstPtr &msg2) {
  ROS_INFO("Received two messages: (%f,%f,%f) and (%f,%f,%f)",
           msg1->vector.x, msg1->vector.y, msg1->vector.z,
           msg2->vector.x, msg2->vector.y, msg2->vector.z);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "subscriber_sync");
  ros::NodeHandle n;
  message_filters::Subscriber<geometry_msgs::Vector3Stamped> sub1(n, "topic1", 1);
  message_filters::Subscriber<geometry_msgs::Vector3Stamped> sub2(n, "topic2", 1);

  // typedef
  // message_filters::sync_policies::ExactTime<geometry_msgs::Vector3Stamped,
  // geometry_msgs::Vector3Stamped> MySyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime
      <geometry_msgs::Vector3Stamped, geometry_msgs::Vector3Stamped>
        MySyncPolicy;

  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub1, sub2);
  sync.registerCallback(boost::bind(&callback, _1, _2));
  ros::spin();
  return 0;
}
