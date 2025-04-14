#include "ros/ros.h"
#include "std_msgs/String.h"

class PubSubNode {
private:
    // ROS node handle
    ros::NodeHandle nh_;

    // Subscribers for the "/chatter" and "/chatter2" topics
    ros::Subscriber chatter_sub_;
    ros::Subscriber chatter2_sub_;

    // Publisher for the "/rechatter" topic
    ros::Publisher rechatter_pub_;

    // Timer for periodic publishing
    ros::Timer timer_;

    // Latest received messages from the subscribed topics
    std_msgs::String chatterMsg_;
    std_msgs::String chatter2Msg_;

public:
    // Constructor: sets up subscribers, publisher, and timer
    PubSubNode() {
        // Subscribe to topics with a queue size of 1
        chatter_sub_ = nh_.subscribe("/chatter", 1, &PubSubNode::chatterCallback, this);
        chatter2_sub_ = nh_.subscribe("/chatter2", 1, &PubSubNode::chatter2Callback, this);

        // Advertise the publisher on "/rechatter" topic
        rechatter_pub_ = nh_.advertise<std_msgs::String>("/rechatter", 1);

        // Create a timer that triggers every 1 second to publish messages
        timer_ = nh_.createTimer(ros::Duration(1.0), &PubSubNode::timerCallback, this);
    }

    // Callback function for the "/chatter" topic
    void chatterCallback(const std_msgs::String::ConstPtr &msg) {
        chatterMsg_ = *msg;
    }

    // Callback function for the "/chatter2" topic
    void chatter2Callback(const std_msgs::String::ConstPtr &msg) {
        chatter2Msg_ = *msg;
    }

    // Timer callback function: publishes messages periodically
    void timerCallback(const ros::TimerEvent &) {
        // Publish the latest messages from both subscribers
        rechatter_pub_.publish(chatterMsg_);
        rechatter_pub_.publish(chatter2Msg_);
        ROS_INFO("Timer callback executed: messages published.");
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "subscribe_and_publish");
    PubSubNode node;
    ros::spin();
    return 0;
}
