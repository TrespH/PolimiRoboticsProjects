#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <fstream>
#include <iomanip>

class GoalRecorder {
public:
    GoalRecorder(ros::NodeHandle& nh) {
        nh.param<std::string>("csv_file", csv_file_, "goals.csv");
        file_.open(csv_file_, std::ios::out | std::ios::trunc);
        if (!file_.is_open()) {
            ROS_ERROR("Failed to open CSV file: %s", csv_file_.c_str());
            ros::shutdown();
        }

        sub_ = nh.subscribe("/move_base_simple/goal", 10, &GoalRecorder::goalCallback, this);
        ROS_INFO("Goal Recorder started. Click goals in RViz.");
    }

    ~GoalRecorder() {
        if (file_.is_open()) {
            file_.close();
            ROS_INFO("CSV file closed.");
        }
    }

private:
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        double x = msg->pose.position.x;
        double y = msg->pose.position.y;

        tf::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w
        );
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        double theta_deg = yaw * 180.0 / M_PI;

        file_ << std::fixed << std::setprecision(2)
              << x << "," << y << "," << theta_deg << std::endl;

        ROS_INFO("Goal saved: x=%.2f, y=%.2f, theta=%.2f", x, y, theta_deg);
    }

    std::ofstream file_;
    std::string csv_file_;
    ros::Subscriber sub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "goal_recorder");
    ros::NodeHandle nh("~");
    GoalRecorder recorder(nh);
    ros::spin();
    return 0;
}
