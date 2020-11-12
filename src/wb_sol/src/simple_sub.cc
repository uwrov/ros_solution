#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include "./simple_sub.h"

int main(int argc, char **argv) {
    // Set up node
    ros::init(argc, argv, "sampleListener");
    ros::NodeHandle n;  // How we can interface with this node

    // Set up subscriber
    std::string subName = "/wheely_boi/wheely_boi/cmd";
    ros::Subscriber sub = n.subscribe(subName, 10, chatterCallback);

    // Don't go past this point so long as ros node is alive
    ros::spin();

    // Return success
    return 0;
}

void chatterCallback(const geometry_msgs::Twist& t) {
    ROS_INFO("Linear: (%f, %f)", t.linear.x, t.linear.z);
    ROS_INFO("Angular: (%f, %f)\n", t.angular.x, t.angular.z);
}
