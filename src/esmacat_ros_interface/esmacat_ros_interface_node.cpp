#include "esmacat_ros_interface_node.h"
#include "esmacat_shared_memory_comm.h"

using namespace  std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "esmacat_ros_interface");
    ros::NodeHandle nh;

    ROS_INFO("esmacat_ros_interface_node launched");

    esmacat_ros_interface ros_interface;

    ros::MultiThreadedSpinner spinner(1); // Use 4 threads
    spinner.spin();


//    ros::Publisher chatter_pub = nh.advertise<std_msgs::Int64>("loop_cnt", 1000);

//    ros::Rate loop_rate(100);
//    while (ros::ok())
//    {
//        std_msgs::Int64 msg;
//        msg.data = esmacat_sm.data->loop_cnt;
//        chatter_pub.publish(msg);

//        ros::spinOnce();

//        loop_rate.sleep();

//        if (esmacat_sm.data->stop)
//        {
//            ros::shutdown();
//            break;
//        }
//    }
    ROS_INFO("esmacat_ros_interface_node killed");

    return 0;


}
