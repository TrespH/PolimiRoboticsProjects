#include <ros/ros.h>
#include <string>

int main(int argc, char **argv) {
    // Initialize the ROS node with the name "my_node"
    ros::init(argc, argv, "my_node");

    // Create a NodeHandle for global parameters
    ros::NodeHandle nh;

    // Create a NodeHandle in the private namespace for node-specific parameters
    ros::NodeHandle private_nh("~");

    // Variables to hold the parameter values
    std::string global_param_value;
    std::string private_param_value;

    // Retrieve a global parameter (set using an absolute name like "/global_param")
    if (nh.getParam("/global_param", global_param_value))
        ROS_INFO("Global parameter: %s", global_param_value.c_str());
    else {
        ROS_WARN("Global parameter not found, using default value");
        global_param_value = "default_global";
    }

    // Retrieve a private (node-specific) parameter (set using the private namespace, e.g., "~private_param")
    if (private_nh.getParam("private_param", private_param_value))
        ROS_INFO("Private parameter: %s", private_param_value.c_str());
    else {
        ROS_WARN("Private parameter not found, using default value");
        private_param_value = "default_private";
    }

    // The node can now use these parameters as needed
    ros::spin();
    return 0;
}
