#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <tf/transform_datatypes.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct Goal {
    double x, y, theta;
};

std::vector<Goal> readGoalsFromCSV(const std::string& filepath) {
    std::vector<Goal> goals;
    std::ifstream file(filepath);
    std::string line;

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string x_str, y_str, theta_str;
        if (std::getline(ss, x_str, ',') &&
            std::getline(ss, y_str, ',') &&
            std::getline(ss, theta_str, ',')) {
            Goal g;
            g.x = std::stod(x_str);
            g.y = std::stod(y_str);
            g.theta = std::stod(theta_str);
            goals.push_back(g);
        }
    }
    return goals;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "navigation");
    ros::NodeHandle nh("~");

    std::string csv_file;
    nh.param<std::string>("csv_file", csv_file, "/home/user/catkin_ws/src/second_project/csv/goals.csv");

    std::vector<Goal> goals = readGoalsFromCSV(csv_file);

    MoveBaseClient ac("move_base", true);
    ROS_INFO("Waiting for move_base action server...");
    ac.waitForServer();
    ROS_INFO("Connected to move_base.");

    for (size_t i = 0; i < goals.size(); ++i) {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = goals[i].x;
        goal.target_pose.pose.position.y = goals[i].y;

        tf::Quaternion q = tf::createQuaternionFromYaw(goals[i].theta);
        tf::quaternionTFToMsg(q, goal.target_pose.pose.orientation);

        ROS_INFO("Sending goal %lu: x=%.2f, y=%.2f, theta=%.2f", i + 1, goals[i].x, goals[i].y, goals[i].theta);
        ac.sendGoal(goal);

        ac.waitForResult();

        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Goal %lu reached!", i + 1);
        else
            ROS_WARN("Failed to reach goal %lu", i + 1);
    }

    return 0;
}