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
    std::vector<std::pair<double,double>> checkpoints = { //coordinates taken from OpenStreetMap
        {45.616364, 9.280795},  // Checkpoint 0 (Finish line). The finish line is before the actual end of the car’s run
        {45.630136, 9.290665},  // Checkpoint 1
        {45.622962, 9.286304}   // Checkpoint 2
    };
    first_project::sector_times sector_msg;
    ros::Time startTime;
    ros::Time now;
    int speedCount;
    int currentSector;
    double speedSum;
    double mean_speed;
    double sectorTime;
    const double R = 6371000.0; //heart radius
    const double CHECKPOINT_TOLERANCE = 5.0;
    bool is_started;

public:
    SectorTimesNode() : currentSector(1), speedSum(0.0), speedCount(0), mean_speed(0.0), sectorTime(0.0), is_started(false){
    speedsteer_sub = nh.subscribe("/speedsteer", 10, &SectorTimesNode::speedCallback, this);
    gps_pose_sub = nh.subscribe("/swiftnav/front/gps_pose", 10, &SectorTimesNode::gpsCallback, this);

    sector_times_pub = nh.advertise<first_project::sector_times>("/sector_times", 10);  
    }

    void speedCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        double speed = msg->point.y; //speed in km/h

        if (is_started && msg->header.stamp < startTime) {   //to launch again rosbag without re-launching roscore
            ROS_WARN("Detected rosbag re-launch ---> data reset)");
            reset();
        }

        if (is_started){ 
            speedSum += speed;
            speedCount++;
            now = msg->header.stamp;
            sectorTime = (now - startTime).toSec();
        } else { //first message received - initialize race start conditions
            startTime = msg->header.stamp;   
            speedSum = speed;
            speedCount = 1;
            is_started = true;    
            ROS_INFO("START!");    
        }

        mean_speed = (speedCount > 0) ? (speedSum / speedCount) : 0.0;  //to avoid errors
        sector_msg.current_sector = currentSector;
        sector_msg.current_sector_time = sectorTime;
        sector_msg.current_sector_mean_speed = mean_speed;
        sector_times_pub.publish(sector_msg);
    }

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        double curr_lat = msg->latitude;
        double curr_lon = msg->longitude;

        if (is_started){
            checkSectorTransition(curr_lat, curr_lon);
        }
    }

    void checkSectorTransition(double curr_lat, double curr_lon){
        int target_checkpoint = currentSector % 3; //checkpoint that follows the current sector (1-->1, 2-->2, 3-->0)
        double target_lat = checkpoints[target_checkpoint].first; //checkpoint latitude
        double target_lon = checkpoints[target_checkpoint].second; //checkpoint longitude
        double dist_to_cp = calculateDistance (curr_lat, curr_lon, target_lat, target_lon); 

        if (dist_to_cp < CHECKPOINT_TOLERANCE){   //if I'm in the neighborhood of the checkpoint        
            ROS_INFO("Sector %d: Time=%.2fs, Speed=%.2f km/h", currentSector, sectorTime, mean_speed); //data print

            startTime = now;
            currentSector = (currentSector % 3) + 1; //sector switching
            speedSum = 0.0;
            speedCount = 0;

            if (currentSector == 1){
                ROS_INFO("LAP COMPLETED!"); //the transition from sector 3 to sector 1 corresponds to the arrival
            }
        }
    }

    double calculateDistance (double lat1, double lon1, double lat2, double lon2){ //Haversine formula: from coordinates to meters (good for short distances)
        double dlat = (lat1 - lat2) * M_PI / 180; //delta lat in radiant
        double dlon = (lon1 - lon2) * M_PI / 180; //delta lon in radiant
        double a = sin(dlat/2)*sin(dlat/2)+cos(lat1*M_PI/180)*cos(lat2*M_PI/180)*sin(dlon/2)*sin(dlon/2);
        double b = std::asin(sqrt(a));
        double dist = 2 * R * b;
        return dist;
    }

    void reset() {
        speedSum = 0.0;
        speedCount = 0;
        mean_speed = 0.0;
        sectorTime = 0.0;
        currentSector = 1;
        is_started = false;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "sector_times");
    SectorTimesNode node;
    ros::spin();
    return 0;
}