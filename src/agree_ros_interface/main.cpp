#include "ros/ros.h"
#include "agree_ros_interface.h"

using namespace  std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "agree_shared_memory_interface_node");
    ros::NodeHandle nh;

    ROS_INFO("AGREE ROS-SHM Interface node launched");

    esmacat_ros_interface_class ros_interface;

    ros::MultiThreadedSpinner spinner(4); // Use 4 threads
    spinner.spin();

    return 0;


}
