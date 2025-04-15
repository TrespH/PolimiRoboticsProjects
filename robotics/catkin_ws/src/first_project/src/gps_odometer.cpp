#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#define a 6378137
// 1- (6356752^2/6378137^2) =~ 0.00669447819799328602965141827689106582391364416821264027072703763869490375...
#define e_2 0.00669447819

class GpsOdometer {
public:
  GpsOdometer() {
    // Initialize the publisher and subscriber
    pub = n.advertise<nav_msgs::Odometry>("gps_odom", 1000);
    sub = n.subscribe("swiftnav/front/gps_pose", 1000, &GpsOdometer::callback, this);
  }

  void callback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    double lat = msg->latitude;
    double lon = msg->longitude;
    double alt = msg->altitude;
    ROS_INFO("Heard front (lat, lon, alt): (%f, %f, %f)", lat, lon, alt);

    nav_msgs::Odometry pub_msg;
	// Ensure that lat_ref, lon_ref, and the input lat, lon are in radians, not degrees. If they are in degrees, convert them using:
	// double lat_rad = lat * M_PI / 180.0;
	// double lon_rad = lon * M_PI / 180.0;
	
    double x_ECEF = 0, y_ECEF = 0, z_ECEF = 0;
    double x_ENU = 0, y_ENU = 0, z_ENU = 0;

    // latLonToECEF
    double N = a / (sqrt((double)(1.0) - e_2 * pow(sin(lat), 2)));
    x_ECEF = (N + alt) * cos(lat) * cos(lon);
    y_ECEF = (N + alt) * cos(lat) * sin(lon);
    x_ECEF = (N * (1 - e_2) + alt) * sin(lat);

    // ECEFToENU
    double x_diff = x_ECEF - x_ECEF_ref;
    double y_diff = y_ECEF - y_ECEF_ref;
    double z_diff = z_ECEF - z_ECEF_ref;
    x_ENU = -sin(lon_ref) * x_diff + cos(lon_ref) * y_diff;
    y_ENU = -sin(lat_ref) * cos(lon_ref) * x_diff -
            sin(lat_ref) * sin(lon_ref) * y_diff + cos(lat_ref) * z_diff;
    z_ENU = cos(lat_ref) * cos(lon_ref) * x_diff +
            cos(lat_ref) * sin(lon_ref) * y_diff + sin(lat_ref) * z_diff;

    pub_msg.pose.pose.position.x = x_ENU;
    pub_msg.pose.pose.position.y = y_ENU;
    pub_msg.pose.pose.position.z = z_ENU;

    ROS_INFO("Pub front (lat, lon, alt): (%f, %f, %f)",
             pub_msg.pose.pose.position.x, pub_msg.pose.pose.position.y,
             pub_msg.pose.pose.position.z);

    pub.publish(pub_msg);
  }

private:
  ros::NodeHandle n;
  ros::Publisher pub;
  ros::Subscriber sub;
  double x_ECEF_ref = 0, y_ECEF_ref = 0, z_ECEF_ref = 0;
  double lat_ref = 0, lon_ref = 0, alt_ref = 0;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "gps_odometer");
  GpsOdometer gpsOdometer;
  ros::spin();
  return 0;
}
