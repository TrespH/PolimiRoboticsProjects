#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <first_project/sector_times.h>
#include <cmath>
#include <vector>


class SectorTimesNode {
private:
    ros::NodeHandle nh;
    ros::Subscriber speedsteer_sub;
    ros::Subscriber gps_pose_sub;
    ros::Publisher sector_times_pub;
    std::vector<std::pair<double,double>> checkpoints = {
        {45.6216, 9.2811},  // Start/Finish
        {45.6235, 9.2880},  // Checkpoint 1
        {45.6170, 9.2745}   // Checkpoint 2
    };
    first_project::sector_times sector_msg;
    ros::Time startTime;
    int speedCount;
    int currentSector;
    double sector_distance;
    double speedSum;
    double prev_lat, prev_lon;
    bool has_prev_pos;
    const double R = 6371000.0;

public:
    SectorTimesNode() : has_prev_pos(false), currentSector(-1) {
    speedsteer_sub = nh.subscribe("/speedsteer", 10, &SectorTimesNode::speedCallback, this);
    gps_pose_sub = nh.subscribe("/swiftnav/front/gps_pose", 10, &SectorTimesNode::gpsCallback, this);

    sector_times_pub = nh.advertise<first_project::sector_times>("/sector_times", 10);  
    }

    void speedCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        double speed = msg->point.y;
        speedSum += speed;
        speedCount++;
    }

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        double curr_lat = msg->latitude;
        double curr_lon = msg->longitude;

        if (currentSector == -1){ 
            initialSector(curr_lat, curr_lon);
        }else{
            if (has_prev_pos){
                double distance = calculateDistance(curr_lat, curr_lon, prev_lat, prev_lon);
                sector_distance += distance;
            }
        }
        prev_lat = curr_lat;
        prev_lon = curr_lon;
        has_prev_pos = true;

        checkSectorTransition(curr_lat, curr_lon);
    }

    double calculateDistance (double lat1, double lon1, double lat2, double lon2){
        double dlat = (lat1 - lat2) * M_PI / 180; //delta lat e lon in radiant
        double dlon = (lon1 - lon2) * M_PI / 180; 
        double a = sin(dlat/2)*sin(dlat/2)+cos(lat1*M_PI/180)*cos(lat2*M_PI/180)*sin(dlon/2)*sin(dlon/2);
        double b = std::asin(sqrt(a));
        double dist = 2 * R * b;
        return dist;
    }

    void initialSector(double lat, double lon){
        for (int i = 0; i < 3; ++i) {
            double dist = calculateDistance(lat, lon, checkpoints[i].first, checkpoints[i].second);
            if (dist < 2.5){
                if (i==0) currentSector = 1;
                if (i==1) currentSector = 2;
                if (i==2) currentSector = 3;
                startTime = ros::Time::now();
                sector_distance = 0.0;
                speedSum = 0.0;
                speedCount = 0;
                ROS_INFO ("Sector founded: %d", currentSector);
                return;
            }
        }
    }

    void checkSectorTransition(double curr_lat, double curr_lon){
        int target_checkpoint = (currentSector % 3) + 1;
        double target_lat = checkpoints[target_checkpoint].first;
        double target_lon = checkpoints[target_checkpoint].second;
        double dist_to_cp = calculateDistance (curr_lat, curr_lon, target_lat, target_lon);

        if (dist_to_cp < 2.5){    //if I'm in the neighborhood of the checkpoint
            ros::Time now = ros::Time::now();
            double sectorTime = (now - startTime).toSec();
            double mean_speed = speedSum / speedCount;

            sector_msg.current_sector = currentSector;
            sector_msg.current_sector_time = sectorTime;
            sector_msg.current_sector_mean_speed = mean_speed;
            sector_times_pub.publish(sector_msg);

            currentSector = (currentSector + 1) % 3;
            startTime = now;
            sector_distance = 0.0;
            speedSum = 0.0;
            speedCount = 0;

            ROS_INFO("Entering in the sector number %d", currentSector);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "sector_times");
    SectorTimesNode node;
    ros::spin();
    return 0;
}