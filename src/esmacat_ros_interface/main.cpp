#include "ros/ros.h"
#include "esmacat_ros_interface.h"

using namespace  std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "esmacat_ros_interface");
    ros::NodeHandle nh;

    ROS_INFO("esmacat_ros_interface node launched");

    esmacat_ros_interface_class ros_interface;

    ros::MultiThreadedSpinner spinner(4); // Use 4 threads
    spinner.spin();

    return 0;


}
