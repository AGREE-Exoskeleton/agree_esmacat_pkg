#include "esmacat_ros_interface.h"
#include "ros/ros.h"
#include <string.h>
#include <stdio.h>

/************************/
/* ROS Publisher Thread */
/************************/

void esmacat_ros_interface_class::ROS_publish_thread(){


  //Declare a message and setup the publisher for that message
  //  esmacat_ros_interface::esmacat_command command;
  //    std_msgs::Int64 msg;
  agree_esmacat_pkg::agree_esmacat_status msg;
  ros::NodeHandle n;
  ros::Rate loop_rate(100);
  ros::Publisher publisher = n.advertise<agree_esmacat_pkg::agree_esmacat_status>("esmacat/status", 1000);


  //Variables that setup the publishing loop
  int interim_roscount = 0;

  while (ros::ok()){

    //    command.setpoint = (int64_t) 100*sin((2.0*3.14159)*interim_roscount/100.0);
    //    command.state = interim_state
    esmacat_sm.data->elapsed_time++;
    msg.elapsed_time = esmacat_sm.data->elapsed_time+1;
    msg.mode         = esmacat_sm.data->state;

    msg.encoder_position.clear();
    msg.encoder_position.push_back(1);
    msg.encoder_position.push_back(2);
    msg.encoder_position.push_back(3);
    msg.encoder_position.push_back(4);

    //sin(2*M_PI*msg.elapsed_time/100));
    msg.loadcell_torque[0]  = esmacat_sm.data->elapsed_time+1;
    msg.loadcell_torque[1]  = -esmacat_sm.data->elapsed_time+1;
    msg.loadcell_torque[2]  = 0.0;
    msg.loadcell_torque[3]  = sin(2*M_PI*msg.elapsed_time/100);

    for(int i=0;i<5;i++){
      msg.loadcell_torque[i]  = esmacat_sm.data->elapsed_time*i;
    }



    publisher.publish(msg);
//    ROS_INFO("Publisher: loop_cnt %f",msg.elapsed_time);
//    if(esmacat_sm.data->state == 0)
//    {
//      ROS_INFO("Hard Real-Time Node Killed");
//      ros::shutdown();
//    }
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

void esmacat_ros_interface_class::ROS_subscribe_thread(){

  //Setup a subscriber that will get data from other ROS nodes
  ros::MultiThreadedSpinner spinner(1); // Use 4 threads

  ros::NodeHandle n;

  ros::Subscriber subscriber = n.subscribe("esmacat/command", 1000, &esmacat_ros_interface_class::ROS_subscribe_callback, this);

  spinner.spin();
}

void esmacat_ros_interface_class::ROS_subscribe_callback(const agree_esmacat_pkg::agree_esmacat_command msg)
{
  //Display data from hard real-time loop to the the terminal.
  esmacat_sm.data->state =  msg.mode;
  if(prev_state != esmacat_sm.data->state)
  {
    ROS_INFO("Change MODE to: %s",state_labels[msg.mode].c_str());
  };

  if(prev_damping != msg.damping_d){
    ROS_INFO("Change DAMPING to: %f",msg.damping_d);
    esmacat_sm.data->joint_controller.impedance_control_d_gain_mNm_per_rad_per_sec = msg.damping_d;
  }

  if(prev_stiffness != msg.stiffness_k){
    ROS_INFO("Change STIFFNESS to: %f",msg.stiffness_k);
    esmacat_sm.data->joint_controller.impedance_control_k_gain_mNm_per_rad = msg.stiffness_k;
  }

  prev_state = msg.mode;
  prev_stiffness = msg.stiffness_k;
  prev_damping   = msg.damping_d;
}

void esmacat_ros_interface_class::print_command_keys()
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
