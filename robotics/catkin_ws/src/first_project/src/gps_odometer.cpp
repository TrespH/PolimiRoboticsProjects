#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include <cmath>
#include <tf/transform_broadcaster.h>

class GpsOdometer {
public:
  GpsOdometer(double lat, double lon, double alt) {
    // Initialize reference point and convert degrees to radians
    lat_ref = lat * deg_to_rad;
    lon_ref = lon * deg_to_rad;
    alt_ref = alt;
    sin_lat_ref = sin(lat_ref);
    cos_lat_ref = cos(lat_ref);
    sin_lon_ref = sin(lon_ref);
    cos_lon_ref = cos(lon_ref);

    // Initialize ECEF reference point
    double N = a / (sqrt(1 - e_2 * sin_lat_ref * sin_lat_ref));
    x_ECEF_ref = (N + alt_ref) * cos_lat_ref * cos_lon_ref;
    y_ECEF_ref = (N + alt_ref) * cos_lat_ref * sin_lon_ref;
    z_ECEF_ref = (N * (1.0 - e_2) + alt_ref) * sin_lat_ref;

    ROS_INFO("Initialized reference: (%f, %f, %f)", lat_ref, lon_ref, alt_ref);
    ROS_INFO("Initialized reference trigon.: (%f, %f, %f, %f)", sin_lat_ref,
             cos_lat_ref, sin_lon_ref, cos_lon_ref);
    ROS_INFO("Initialized ECEF reference: (%f, %f, %f)", x_ECEF_ref, y_ECEF_ref,
             z_ECEF_ref);

    // Initialize the publisher and subscriber
    pub = n.advertise<nav_msgs::Odometry>("gps_odom", 1000);
    sub = n.subscribe("swiftnav/front/gps_pose", 1000, &GpsOdometer::callback,
                      this);
  }

  void callback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    ros::Time timestamp = msg->header.stamp;

    const double lat = msg->latitude * deg_to_rad;
    const double lon = msg->longitude * deg_to_rad;
    const double alt = msg->altitude;
    ROS_INFO("Heard front (lat, lon, alt): (%f, %f, %f)", lat, lon, alt);

    // lat-lon-alt To ECEF
    const double N = a / (sqrt(1.0 - e_2 * sin(lat) * sin(lat)));
    double x_ECEF = (N + alt) * cos(lat) * cos(lon);
    double y_ECEF = (N + alt) * cos(lat) * sin(lon);
    double z_ECEF = (N * (1.0 - e_2) + alt) * sin(lat);

    // ECEF to ENU
    double x_diff = x_ECEF - x_ECEF_ref;
    double y_diff = y_ECEF - y_ECEF_ref;
    double z_diff = z_ECEF - z_ECEF_ref;
    double x_ENU = -sin_lon_ref * x_diff + cos_lon_ref * y_diff;
    double y_ENU = -sin_lat_ref * cos_lon_ref * x_diff -
                   sin_lat_ref * sin_lon_ref * y_diff + cos_lat_ref * z_diff;
    double z_ENU = cos_lat_ref * cos_lon_ref * x_diff +
                   cos_lat_ref * sin_lon_ref * y_diff + sin_lat_ref * z_diff;

    nav_msgs::Odometry pub_msg;
    pub_msg.pose.pose.position.x = x_ENU;
    pub_msg.pose.pose.position.y = y_ENU;
    pub_msg.pose.pose.position.z = z_ENU;
    pub_msg.header.stamp = timestamp;
    pub_msg.header.frame_id = "odom";
    pub_msg.child_frame_id = "gps";

    ROS_INFO("Pub front ENU: (%f, %f, %f)",
             pub_msg.pose.pose.position.x, pub_msg.pose.pose.position.y,
             pub_msg.pose.pose.position.z);

    pub.publish(pub_msg);

    transform.setOrigin(tf::Vector3(x_ENU, y_ENU, z_ENU));
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "gps"));
  }

private:
  ros::NodeHandle n;
  ros::Publisher pub;
  ros::Subscriber sub;
  tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;

  const double a = 6378137.0; // semi major axis of the equatorial radius
  const double b = 6356752.0; // semi minor axis of the polar radius
  // 1- (6356752^2/6378137^2) =~
  // 0.00669447819799328602965141827689106582391364416821264027072703763869490375...
  const double e_2 = 1.0 - pow(b, 2) / pow(a, 2);
  const double deg_to_rad = M_PI / 180.0;

  double x_ECEF_ref, y_ECEF_ref, z_ECEF_ref;
  double lat_ref, lon_ref, alt_ref;
  double sin_lat_ref, cos_lat_ref, sin_lon_ref, cos_lon_ref;
};

int main(int argc, char **argv) {
  if (argc < 4) {
    ROS_ERROR("Usage: %s <latitude> <longitude> <altitude>", argv[0]);
    return 1;
  }
  ros::init(argc, argv, "gps_odometer");
  ROS_INFO("Data from argv: (%s, %s, %s)", argv[1], argv[2], argv[3]);
  GpsOdometer gpsOdometer =
      GpsOdometer(atof(argv[1]), atof(argv[2]), atof(argv[3]));
  ros::spin();
  return 0;
}
