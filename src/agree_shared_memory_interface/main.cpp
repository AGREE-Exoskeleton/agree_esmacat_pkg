#include "ros/ros.h"
#include "agree_shared_memory_comm.h"

using namespace  std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "agree_shared_memory_check_node");
    ros::NodeHandle nh;

    ROS_INFO("AGREE ROS-SHM Check node launched");

    agree_shared_memory_comm agree_shm;

    // Initializing the shared memory
    if (agree_shm.init())
    {
        int key = agree_shm.get_shared_memory_key(); // get the shm key to be compared with "ipcs -m"
        ROS_INFO("AGREE ROS-SHM Check shared memory initialized with key 0x%x", key);
    }
    else
    {
        cout << "AGREE ROS-SHM Check shared memory initialization has been failed";
        agree_shm.detach_shared_memory(); // Something went wrong, detach shm
    }

    ros::MultiThreadedSpinner spinner(4); // Use 4 threads
    spinner.spin();

    return 0;


}
