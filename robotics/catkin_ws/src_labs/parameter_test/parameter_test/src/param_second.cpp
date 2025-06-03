#include <ros/ros.h>
#include <dynamic_reconfigure/server.h> // Will act as a server
#include <parameter_test/parametersConfig.h>

void callback(parameter_test::parametersConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure Request: %d %f %s %s %d",
             config.int_param,
             config.double_param,
             config.str_param.c_str(),
             config.bool_param ? "True" : "False",
             config.size);
    ROS_INFO("Level: %d", level);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "param_second");
    dynamic_reconfigure::Server<parameter_test::parametersConfig> server;
    dynamic_reconfigure::Server<parameter_test::parametersConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2); // Called as soon as params change
    server.setCallback(f);
    ROS_INFO("Spinning node");
    ros::spin();
    return 0;
}
