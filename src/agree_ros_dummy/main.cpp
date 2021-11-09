#include "ros/ros.h"
#include "agree_ros_dummy.h"

using namespace  std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "agree_ros_simulation_node");
    ros::NodeHandle nh;

    ROS_INFO("AGREE Simulation node launched");

    esmacat_ros_interface_class ros_interface;

    // Create timer for ROS parameters
    ros::Timer timer = nh.createTimer(ros::Duration(1.0), &esmacat_ros_interface_class::ROS_parameters_callback, &ros_interface);

    // Create subscriber for ROS commands
    ros::Subscriber subscriber = nh.subscribe("esmacat/command", 1000, &esmacat_ros_interface_class::ROS_subscribe_callback, &ros_interface);

    ros::MultiThreadedSpinner spinner(4); // Use 4 threads
    spinner.spin();

    return 0;

}

