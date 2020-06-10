#include "ros_interface.h"
#include "ros/ros.h"

/************************/
/* ROS Publisher Thread */
/************************/

void esmacat_ros_interface::ROS_publish_thread(){


    //Declare a message and setup the publisher for that message
    //  esmacat_ros_interface::esmacat_command command;
    std_msgs::Int64 msg;
    ros::NodeHandle n;
    ros::Rate loop_rate(100);
    ros::Publisher publisher = n.advertise<std_msgs::Int64>("esmacat_sensor", 1000);


    //Variables that setup the publishing loop
    int interim_roscount = 0;

    while (ros::ok()){

        //    command.setpoint = (int64_t) 100*sin((2.0*3.14159)*interim_roscount/100.0);
        //    command.state = interim_state;
        msg.data = (int64_t)esmacat_sm.data->loop_cnt;
        publisher.publish(msg);
        //      ROS_INFO("Publisher: loop_cnt %d",msg.data);
        //    if(esmacat_sm.data->stop == true) ros::shutdown();

        loop_rate.sleep();
        interim_roscount++;
        if (esmacat_sm.data->stop)
        {
            ROS_INFO("esmacat_ros_interface_node killed");
            ros::shutdown();
            break;

        }
    }

}

/************************/
/* ROS Subscriber Thread */
/************************/

void esmacat_ros_interface::ROS_subscribe_thread(){

    //Setup a subscriber that will get data from other ROS nodes
    ros::MultiThreadedSpinner spinner(1); // Use 4 threads

    ros::NodeHandle n;

    ros::Subscriber subscriber = n.subscribe("esmacat_command", 1000, &esmacat_ros_interface::ROS_subscribe_callback, this);

    spinner.spin();
}

void esmacat_ros_interface::ROS_subscribe_callback(const std_msgs::Int64 msg)
{
    //Display data from hard real-time loop to the the terminal.
    esmacat_sm.data->state =  msg.data;
    if(prev_state != esmacat_sm.data->state)
    {

        ROS_INFO("Change State to: %c", &state_labels[esmacat_sm.data->state]);
    };
    prev_state = esmacat_sm.data->state;
    //std::cout << esmacat_sm.data->state << endl;
}

void esmacat_ros_interface::print_command_keys()
{
    std::cout << boldred_key << "\nCOMMAND KEYS:"<< color_key << std::endl;
    std::cout << blue_key << "\'k\'" << color_key << ": exit" << "\n";
    std::cout << blue_key << "\'s\'" << color_key << ": STOP mode"<< "\n";
    std::cout << blue_key << "\'c\'" << color_key << ": CURRENT mode"<< "\n";
    std::cout << blue_key << "\'t\'" << color_key << ": TORQUE mode"<< "\n";
    std::cout << blue_key << "\'n\'" << color_key << ": NULL-TORQUE mode" << "\n";
    std::cout << blue_key << "\'g\'" << color_key << ": GRAVITY mode"<< "\n";
    std::cout << blue_key << "\'f\'" << color_key << ": FREEZE mode"<< "\n";
    std::cout << blue_key << "\'ENTER\'" << color_key << ": SHOW current settings and command keys\n"<< "\n";
}
