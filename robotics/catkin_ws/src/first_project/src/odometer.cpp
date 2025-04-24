#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <cmath>

class Odometer {
public:
  Odometer() {
    x = 0.0;
    y = 0.0;
    theta = 1.47; // Initial GPS heading measurement

    ros::NodeHandle n;
    sub = n.subscribe("/speedsteer", 10, &Odometer::callback, this);
    pub = n.advertise<nav_msgs::Odometry>("/odom", 10);
	
	// Setup timer for publishing transforms at fixed rate (e.g., 1Hz)
    pub_timer = n.createTimer(ros::Duration(1), &Odometer::publishTf, this);
	
	// Initialize flag
    have_data = false;
  }

  void callback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    double speed_kmh = msg->point.y; // Vehicle speed in km/h
    double steering_wheel_deg  = msg->point.x; // Steering wheel angle in degrees
    steering_wheel_deg += STEER_ADJUST_FACTOR;
    ros::Time current_time = msg->header.stamp;

    if (last_time.isZero()) {
      last_time = current_time;
      return;
    }

    double dt = (current_time - last_time).toSec();
    last_time = current_time;

    // Convert units
    double speed_ms = speed_kmh * KMH_TO_MS;
    double wheel_rad = (steering_wheel_deg  / STEERING_FACTOR) * DEG_TO_RAD;


    // R = d / tan(α) + b
    double R = (FRONT_REAR_WHEELS_DISTANCE / tan(wheel_rad)) + HALF_REAR_WHEELS_DISTANCE;

    // Angular velocity
    double omega = speed_ms / R;

    x += speed_ms * cos(theta) * dt;
    y += speed_ms * sin(theta) * dt;
    theta += omega * dt;
	
	have_data = true;
    publishOdometry(current_time, speed_ms, omega);
  }
  
  void publishTf(const ros::TimerEvent&) {
	if (!have_data) return;
    // Broadcast transform (odom -> vehicle)
    transform.setOrigin(tf::Vector3(x, y, 0.0));
    q.setRPY(0, 0, theta);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "vehicle"));
  }

private:
  // Vehicle parameters
  const double FRONT_REAR_WHEELS_DISTANCE = 1.765; // Distance between front and rear wheels [m]
  const double HALF_REAR_WHEELS_DISTANCE = 1.30 / 2.0; // (2b/2) [m]
  const double STEERING_FACTOR = 32.0; // Steering wheel to wheel angle ratio
  const double STEER_ADJUST_FACTOR = 7.8; // Misalignent observed at the first rettilineo 
  const double DEG_TO_RAD = M_PI / 180.0;
  const double KMH_TO_MS = 1000.0 / 3600.0;

  ros::Subscriber sub;
  ros::Publisher pub;
  
  tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;

  double x, y, theta;
  ros::Time last_time;

  // Timer variables
  ros::Timer pub_timer;
  bool have_data;

  // Publish odometry data
  void publishOdometry(const ros::Time& stamp, double linear_vel, double angular_vel) {
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "vehicle";

    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0;

    tf::Quaternion temp_q;
    temp_q.setRPY(0, 0, theta);
    odom_msg.pose.pose.orientation.x = temp_q.x();
    odom_msg.pose.pose.orientation.y = temp_q.y();
    odom_msg.pose.pose.orientation.z = temp_q.z();
    odom_msg.pose.pose.orientation.w = temp_q.w();

    odom_msg.twist.twist.linear.x = linear_vel;
    odom_msg.twist.twist.angular.z = angular_vel;

    pub.publish(odom_msg);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "odometer");
  ROS_INFO("Odometer node started with simplified Ackermann model");
  Odometer odomNode;
  ros::spin();
  return 0;
}
