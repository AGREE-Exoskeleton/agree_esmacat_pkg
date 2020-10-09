#include "testbed_ros_interface.h"

int main(int argc, char **argv)
{

  // ROS Initialization
  ros::init(argc, argv, "testbed_ros_interface");

  // ROS Class Object Creation
  testbed_ros_interface testbed_ros;

  // Spinner Configuration
  ros::MultiThreadedSpinner spinner; // Use 4 threads (default)


  // Start Spinner
  spinner.spin();

  return 0;
}
