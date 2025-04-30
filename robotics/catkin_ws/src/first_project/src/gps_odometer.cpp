#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include <cmath>
#include <tf/transform_broadcaster.h>
#include <deque>

#define HEADING_BUFFER_SIZE 5 // Smoothing buffer size

class GpsOdometer {
public:
  GpsOdometer(double lat, double lon, double alt) {
    // Initialize reference point and convert degrees to radians
    lat_ref = lat * DEG_TO_RAD;
    lon_ref = lon * DEG_TO_RAD;
    alt_ref = alt;
    sin_lat_ref = sin(lat_ref);
    cos_lat_ref = cos(lat_ref);
    sin_lon_ref = sin(lon_ref);
    cos_lon_ref = cos(lon_ref);

    // Initialize ECEF reference point
    double N = A / (sqrt(1 - E2 * sin_lat_ref * sin_lat_ref));
    x_ECEF_ref = (N + alt_ref) * cos_lat_ref * cos_lon_ref;
    y_ECEF_ref = (N + alt_ref) * cos_lat_ref * sin_lon_ref;
    z_ECEF_ref = (N * E2_INV + alt_ref) * sin_lat_ref;

    ROS_INFO("Initialized reference: (%f, %f, %f)", lat_ref, lon_ref, alt_ref);
    ROS_INFO("Initialized reference trigon.: (%f, %f, %f, %f)", sin_lat_ref, cos_lat_ref, sin_lon_ref, cos_lon_ref);
    ROS_INFO("Initialized ECEF reference: (%f, %f, %f)", x_ECEF_ref, y_ECEF_ref, z_ECEF_ref);

    // Initialize the publisher and subscriber
    sub = n.subscribe("/swiftnav/front/gps_pose", 1000, &GpsOdometer::callback, this);
    pub = n.advertise<nav_msgs::Odometry>("/gps_odom", 1000);
	
	  // Setup timer for publishing transforms at fixed rate (1Hz)
    pub_timer = n.createTimer(ros::Duration(1), &GpsOdometer::publishTf, this);
    
    // Initialize flag
    have_data = false;
  }

  double smoothHeading(double new_heading) {
    // Handle wrap-around for angles
    if (!heading_buffer.empty()) {
        while (new_heading - heading_buffer.back() > M_PI) new_heading -= 2*M_PI;
        while (new_heading - heading_buffer.back() < -M_PI) new_heading += 2*M_PI;
    }
    
    heading_buffer.push_back(new_heading);
    if (heading_buffer.size() > HEADING_BUFFER_SIZE) {
        heading_buffer.pop_front();
    }
    
    // Calculate average
    double sin_sum = 0, cos_sum = 0;
    for (double h : heading_buffer) {
        sin_sum += sin(h);
        cos_sum += cos(h);
    }
    return atan2(sin_sum/heading_buffer.size(), cos_sum/heading_buffer.size());
  }

  void callback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    ros::Time timestamp = msg->header.stamp;

    const double lat = msg->latitude * DEG_TO_RAD;
    const double lon = msg->longitude * DEG_TO_RAD;
    const double alt = msg->altitude;
    const double sin_lat = sin(lat), cos_lat = cos(lat), sin_lon = sin(lon), cos_lon = cos(lon);
	
    // lat-lon-alt To ECEF
    const double N = A / (sqrt(1.0 - E2 * sin_lat * sin_lat));
    double x_ECEF = (N + alt) * cos_lat * cos_lon;
    double y_ECEF = (N + alt) * cos_lat * sin_lon;
    double z_ECEF = (N * E2_INV + alt) * sin_lat;

    // ECEF to ENU
    double x_diff = x_ECEF - x_ECEF_ref;
    double y_diff = y_ECEF - y_ECEF_ref;
    double z_diff = z_ECEF - z_ECEF_ref;
    x_ENU = -sin_lon_ref * x_diff + cos_lon_ref * y_diff;
    y_ENU = -sin_lat_ref * cos_lon_ref * x_diff - sin_lat_ref * sin_lon_ref * y_diff + cos_lat_ref * z_diff;
    z_ENU = cos_lat_ref * cos_lon_ref * x_diff + cos_lat_ref * sin_lon_ref * y_diff + sin_lat_ref * z_diff;

    double raw_heading = atan2(y_ENU - y_ENU_prev, x_ENU - x_ENU_prev);
    smoothed_heading = smoothHeading(raw_heading);

    x_ENU_prev = x_ENU;
    y_ENU_prev = y_ENU;

    have_data = true;
	  publishOdometry(timestamp);
  }

private:
  ros::NodeHandle n;
  ros::Publisher pub;
  ros::Subscriber sub;
  tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;

  const double A = 6378137.0; // semi major axis of the equatorial radius
  const double B = 6356752.0; // semi minor axis of the polar radius
  const double E2 = 1.0 - pow(B, 2) / pow(A, 2); // =~ 0.0066944
  const double E2_INV = 1.0 - E2;
  const double DEG_TO_RAD = M_PI / 180.0;

  double x_ECEF_ref, y_ECEF_ref, z_ECEF_ref;
  double lat_ref, lon_ref, alt_ref;
  double sin_lat_ref, cos_lat_ref, sin_lon_ref, cos_lon_ref;
  double x_ENU, y_ENU, z_ENU, smoothed_heading;
  
  // Timer variables
  ros::Timer pub_timer;
  bool have_data;

  // Initial previous ENU, for initial heading calculation
  double x_ENU_prev = 0, y_ENU_prev = 0;

  // Orientation smoothing
  std::deque<double> heading_buffer;
  
  void publishOdometry(const ros::Time& stamp) {
	  tf::Quaternion temp_q;
    temp_q.setRPY(0, 0, smoothed_heading);
	  nav_msgs::Odometry pub_msg;
    pub_msg.pose.pose.position.x = x_ENU;
    pub_msg.pose.pose.position.y = y_ENU;
    pub_msg.pose.pose.position.z = z_ENU;
    pub_msg.pose.pose.orientation.x = temp_q.x();
    pub_msg.pose.pose.orientation.y = temp_q.y();
    pub_msg.pose.pose.orientation.z = temp_q.z();
    pub_msg.pose.pose.orientation.w = temp_q.w();
    pub_msg.header.stamp = stamp;
    pub_msg.header.frame_id = "odom";
    pub_msg.child_frame_id = "gps";
    pub.publish(pub_msg);
    // ROS_INFO("Pub: (%f, %f, %f); heading: (%f)", x_ENU, y_ENU, z_ENU, smoothed_heading);
  }

  void publishTf(const ros::TimerEvent&) {
    if (!have_data) return;
      // Broadcast transform (odom -> gps)
      transform.setOrigin(tf::Vector3(x_ENU, y_ENU, z_ENU));
      q.setRPY(0, 0, smoothed_heading);
      transform.setRotation(q);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "gps"));
  }
};

int main(int argc, char** argv) {
  if (argc < 4) {
    ROS_ERROR("Launch usage: %s <latitude> <longitude> <altitude>", argv[0]);
    return 1;
  }
  ros::init(argc, argv, "gps_odometer");
  ROS_INFO("Data from argv: (%s, %s, %s)", argv[1], argv[2], argv[3]);
  GpsOdometer gpsOdometer = GpsOdometer(atof(argv[1]), atof(argv[2]), atof(argv[3]));
  ros::spin();
  return 0;
}
