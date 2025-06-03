#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

class OdomToTF {
public:
  OdomToTF() {
    sub = n.subscribe("/odometry", 1000, &OdomToTF::odomCallback, this);
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    geometry_msgs::TransformStamped odom_tf;
    odom_tf.header.stamp = msg->header.stamp;
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_link";
    odom_tf.transform.translation.x = msg->pose.pose.position.x;
    odom_tf.transform.translation.y = msg->pose.pose.position.y;
    odom_tf.transform.translation.z = msg->pose.pose.position.z;
    odom_tf.transform.rotation = msg->pose.pose.orientation;

    br.sendTransform(odom_tf);
  }

private:
  ros::NodeHandle n;
  ros::Subscriber sub;
  tf2_ros::TransformBroadcaster br;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "odom_to_tf");
  OdomToTF odom_to_tf;
  ros::spin();
  return 0;
}