#include "ros/ros.h"
#include "agree_ros_dummy.h"

using namespace  std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "agree_ros_simulation_node");
    ros::NodeHandle nh;

    ROS_INFO("AGREE Simulation node launched");

    int side;
    nh.getParam("side",side);

//    if(side==RIGHT){
//      ROS_INFO_STREAM("Right side selected... importing dummy rest position" << endl);
//      nh.setParam("matlab/point0/configuration/J1",0.70);
//      nh.setParam("matlab/point0/configuration/J2",-1.0);
//      nh.setParam("matlab/point0/configuration/J3",0.15);
//      nh.setParam("matlab/point0/configuration/J4",1.25);
//      nh.setParam("matlab/point0/configuration/J5",0.0);
//    }

    esmacat_ros_interface_class ros_interface;

    // Create timer for ROS parameters
    ros::Timer timer = nh.createTimer(ros::Duration(1.0), &esmacat_ros_interface_class::ROS_parameters_callback, &ros_interface);

    // Create subscriber for ROS commands
    ros::Subscriber subscriber = nh.subscribe("esmacat/command", 1000, &esmacat_ros_interface_class::ROS_subscribe_callback, &ros_interface);

    ros::MultiThreadedSpinner spinner(4); // Use 4 threads
    spinner.spin();

    return 0;

}

