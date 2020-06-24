#include "esmacat_shared_memory_comm.h"
#include "ros/ros.h"
#include "ros_interface.h"

using namespace  std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "esmacat_ros_interface");
    ros::NodeHandle nh;

    ROS_INFO("esmacat_ros_interface node launched");

    esmacat_ros_interface_class ros_interface;

    ros::MultiThreadedSpinner spinner(4); // Use 4 threads
    spinner.spin();


//    ros::Publisher chatter_pub = nh.advertise<std_msgs::Int64>("loop_cnt", 1000);

//    ros::Rate loop_rate(100);
//    while (ros::ok())
//    {
//        std_msgs::Int64 msg;
//        msg.data = esmacat_sm.data->loop_cnt;
//      chatter_pub.publish(msg);

//        ros::spinOnce();

//       loop_rate.sleep();

//        if (ros_interface.esmacat_sm.data->stop)
//        {
//            ROS_INFO("esmacat_ros_interface_node killed");
//            ros::shutdown();

//        }
//    }

    return 0;


}
