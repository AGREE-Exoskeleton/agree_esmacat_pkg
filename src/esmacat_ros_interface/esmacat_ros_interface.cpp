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

    msg.elapsed_time    = esmacat_sm.data->elapsed_time;
    msg.status          = esmacat_sm.data->status;

    msg.joint_position.clear();
    msg.joint_velocity.clear();
    msg.joint_torque.clear();
    msg.setpoint_torque.clear();

    for(int joint_index=0;joint_index<5;joint_index++){
        msg.joint_position.push_back(esmacat_sm.data->joint_status[joint_index].incremental_encoder_position_radians);
        msg.joint_velocity.push_back(esmacat_sm.data->joint_status[joint_index].velocity_rad_per_s);
        msg.joint_torque.push_back(esmacat_sm.data->joint_status[joint_index].loadcell_torque_mNm);
//        msg.setpoint_torque.push_back(esmacat_sm.data->joint_status[joint_index].loadcell_torque_mNm);

    }


    publisher.publish(msg);

    loop_rate.sleep();
    interim_roscount++;
    if (esmacat_sm.data->command == 0)
    {
      ROS_INFO("AGREE ROS Interface shutting down..");
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
  if(prev_command != msg.command)  {
    ROS_INFO("Change MODE to: %s",state_labels[msg.command].c_str());
  }

  if(prev_damping != msg.damping_d[0]){
    ROS_INFO("Change DAMPING to: %f",msg.damping_d[0]);
  }

  if(prev_stiffness != msg.stiffness_k[0]){
    ROS_INFO("Change STIFFNESS to: %f",msg.stiffness_k[0]);
  }

  // Save data from ROS message to shared memory
  esmacat_sm.data->command                                  =  msg.command;
  esmacat_sm.data->robot_config.weight_compensation_level   = msg.weight_assistance;

  for(int joint_index = 0; joint_index < 5; joint_index++){
      esmacat_sm.data->joint_impedance_control_config[joint_index].impedance_control_k_gain_mNm_per_rad         = msg.stiffness_k[joint_index];
      esmacat_sm.data->joint_impedance_control_config[joint_index].impedance_control_d_gain_mNm_per_rad_per_sec = msg.damping_d[joint_index];
      esmacat_sm.data->joint_impedance_control_status[joint_index].impedance_control_setpoint_rad               = msg.setpoint[joint_index];
  }

  prev_command   = msg.command;
  prev_stiffness = msg.stiffness_k[0];
  prev_damping   = msg.damping_d[0];
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
