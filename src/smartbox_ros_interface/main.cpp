#include "interface.h"

int main(int argc, char **argv)
{

  //Initialize ROS
  ros::init(argc, argv, "smartbox_ros_interface");

  smartbox_interface smartbox_ros;
  ros::MultiThreadedSpinner spinner; // Use 4 threads
  spinner.spin();

  return 0;
}
