#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include <tf/transform_broadcaster.h>

class TfSubPub {
public:
  TfSubPub() {
    // Subscribe to the topic and bind the callback method.
    sub = n.subscribe("/turtle1/pose", 1000, &TfSubPub::callback, this);
  }

  void callback(const turtlesim::Pose::ConstPtr &msg) {
    // Update the transform's origin with the new pose
    transform.setOrigin(tf::Vector3(msg->x, msg->y, 0));
    // Update the quaternion based on the new theta value (Roll, Pitch, Yaw)
    q.setRPY(0, 0, msg->theta);
    transform.setRotation(q);
    // Publish the updated transform ( ..., root, childFrame)
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "turtle"));
  }

private:
  ros::NodeHandle n;
  tf::TransformBroadcaster br; // Analog to a publisher
  ros::Subscriber sub;
  // Declare transform and quaternion as class members, to reuse them in each callback
  tf::Transform transform;
  tf::Quaternion q;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "subscribe_and_publish");
  TfSubPub myTfSubPub;
  ros::spin();
  return 0;
}
