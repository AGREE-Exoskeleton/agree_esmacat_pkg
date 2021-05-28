#include "agree_ros_interface.h"
#include "ros/ros.h"
#include <string.h>
#include <stdio.h>

/************************/
/* ROS Publisher Thread */
/************************/

void esmacat_ros_interface_class::ROS_publish_thread(){

  //Declare a message and setup the publisher for that message
  agree_esmacat_pkg::agree_esmacat_status msg;
  ros::NodeHandle n;
  ros::Rate loop_rate(100);
  ros::Publisher publisher = n.advertise<agree_esmacat_pkg::agree_esmacat_status>("esmacat/status", 1000);

  //Variables that setup the publishing loop
  int interim_roscount = 0;

  while (ros::ok()){

    msg.elapsed_time    = esmacat_sm.data->elapsed_time_ms;
    msg.status          = esmacat_sm.data->control_mode_status;

    msg.joint_position_rad.clear();
    msg.joint_speed_rad_s.clear();
    msg.joint_torque_mNm.clear();
    msg.setpoint_torque.clear();

    for(int joint_index=0;joint_index<5;joint_index++){
        msg.joint_position_rad.push_back(esmacat_sm.data->joint_values[joint_index].incremental_encoder_reading_radians);
        msg.joint_speed_rad_s.push_back(esmacat_sm.data->joint_values[joint_index].incremental_encoder_speed_radians_sec);
        msg.joint_torque_mNm.push_back(esmacat_sm.data->joint_values[joint_index].filtered_load_mNm);
        msg.setpoint_torque.push_back(esmacat_sm.data->impedance_control_terms[joint_index].torque_setpoint_mNm);
    }

    publisher.publish(msg);

    loop_rate.sleep();
    interim_roscount++;

    if (esmacat_sm.data->control_mode_command == 0 || esmacat_sm.data->control_mode_status == 0)
    {
      ROS_INFO("AGREE SHM Interface exit conditions met and shutting down..");
      esmacat_sm.detach_shared_memory();
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
  // Save data from ROS message to shared memory
  esmacat_sm.data->control_mode_command                             = (robot_control_mode_t) msg.command;

  // Save weight assistance gains to shared memory
  // TODO: save also second gain
  esmacat_sm.data->arm_weight_compensation_config.weight_assistance = msg.weight_assistance[0];


  for(int joint_index = 0; joint_index < 5; joint_index++){
      esmacat_sm.data->impedance_control_command[joint_index].impedance_control_k_gain_mNm_per_rad         = msg.stiffness_k[joint_index];
      esmacat_sm.data->impedance_control_command[joint_index].impedance_control_d_gain_mNm_per_rad_per_sec = msg.damping_d[joint_index];
      esmacat_sm.data->impedance_control_command[joint_index].impedance_control_setpoint_rad               = msg.setpoint[joint_index];
  }

  //Display data from hard real-time loop to the the terminal.
  if(prev_command != msg.command)  {
    ROS_INFO("Change MODE to: %s",robot_mode_labels[msg.command].c_str());
  }

  if(prev_damping != msg.damping_d[0]){
    ROS_INFO("Change DAMPING to: %f",msg.damping_d[0]);
  }

  if(prev_stiffness != msg.stiffness_k[0]){
    ROS_INFO("Change STIFFNESS to: %f",msg.stiffness_k[0]);
  }

  prev_command   = msg.command;
  prev_stiffness = msg.stiffness_k[0];
  prev_damping   = msg.damping_d[0];
}


/************************/
/* ROS Parameters Thread */
/************************/

void esmacat_ros_interface_class::ROS_parameters_thread(){

  //Setup a subscriber that will get data from other ROS nodes
//  ros::MultiThreadedSpinner spinner(0); // Use 4 threads

  ros::NodeHandle n;
  ros::Rate loop_rate(1);
  // TODO: fix timer to read parameters
//  ros::Timer timer = n.createTimer(ros::Duration(1), timerCallback);

  int mode;
  float lower_soft_stop, upper_soft_stop;
  float weight,height,forearm_length,upperarm_length;
  int side;

  while(ros::ok()){
//  if (n.hasParam("robot_parameters"))
//    {
//      n.getParam("robot_parameters/starting_mode",mode );
//      esmacat_sm.data->control_mode_command = static_cast<robot_control_mode_t>(mode);

//      for (int joint_index=0;joint_index<5;joint_index++){

//          char parameter [50];

//          sprintf(parameter,"robot_parameters/J%d/min_angle",joint_index+1);
//          n.getParam(parameter,lower_soft_stop);

//          sprintf(parameter,"robot_parameters/J%d/max_angle",joint_index+1);
//          n.getParam(parameter,upper_soft_stop);

//          esmacat_sm.data->impedance_control_command[joint_index].soft_stop_lower_limit_rad = lower_soft_stop;
//          esmacat_sm.data->impedance_control_command[joint_index].soft_stop_upper_limit_rad = upper_soft_stop;
//      }

//      ROS_INFO("AGREE Robot Parameters");
//    }
//  else
//  {
//      ROS_ERROR("Failed to get ROS parameters 'robot_parameters'");
//  }

  if (n.hasParam("physiological_param"))
    {
      n.getParam("physiological_param/weight",weight );
      n.getParam("physiological_param/height",height );
      n.getParam("physiological_param/forearm_length",forearm_length );
      n.getParam("physiological_param/upperarm_length",upperarm_length );

      esmacat_sm.data->arm_weight_compensation_config.human_weight_kg  =   weight;
      esmacat_sm.data->arm_weight_compensation_config.human_height_m   =   height;
      esmacat_sm.data->arm_weight_compensation_config.forearm_length_m =   forearm_length;
      esmacat_sm.data->arm_weight_compensation_config.upperarm_length_m =  upperarm_length;
    }
  else
  {
    ROS_ERROR("Failed to get ROS parameters 'user_parameters'");
  }

  if (n.hasParam("side"))
    {
      n.getParam("side",side );
      esmacat_sm.data->arm_weight_compensation_config.side = side;
      cout << side << endl;
    }
  else
  {
    ROS_ERROR("Failed to get ROS parameters 'side'");
  }
  loop_rate.sleep();
  }

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
