#include "agree_ros_dummy.h"
#include "ros/ros.h"
#include <iostream>
#include <string>
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

  //    Inform the real-time thread the system is running through ROS
  ROS_WARN("ROS enabled.");

  while (ros::ok()){

    msg.joint_position_rad.clear();
    msg.joint_speed_rad_s.clear();
    msg.joint_torque_mNm.clear();
    msg.setpoint_torque_mNm.clear();
    msg.setpoint_position_rad.clear();


    for(int joint_index=0;joint_index<5;joint_index++){
      msg.joint_position_rad.push_back(0.0);
      msg.joint_speed_rad_s.push_back(0.0);
      msg.joint_torque_mNm.push_back(0.0);
      msg.setpoint_torque_mNm.push_back(0.0);
      msg.setpoint_position_rad.push_back(0.0);
    }

    msg.status = interim_status;

    publisher.publish(msg);

    loop_rate.sleep();
    interim_roscount++;



    if (interim_command == 0 || interim_status == 0 || ros::master::check() == 0)
    {
      ROS_WARN("ROS disabled.");
      ROS_INFO("AGREE Simulation exit conditions met and shutting down..");
      ros::shutdown();
      break;
    }
  }

}

/************************/
/* ROS Command Thread */
/************************/

void esmacat_ros_interface_class::ROS_command_thread(){

  //Initialize Robot status
  char c;
  string inputString;
  uint8_t state(STOP);

  bool swap_state(false);

  while (ros::ok()){


    // Get character
    c = cin.get();

    if(c != '\n'){
      switch(c){

      case 'k': case 'K':
        std::cout << green_key << "Quick-swapped to new mode!" << color_key << std::endl;
        if(interim_command == robot_control_mode_t::homing_control) interim_status = robot_control_mode_t::homing_done;
        else if(interim_command == robot_control_mode_t::go_position_control) interim_status = robot_control_mode_t::go_position_done;
        else if(interim_command == robot_control_mode_t::shutdown_control) interim_status = robot_control_mode_t::shutdown_done;
        else interim_status = interim_command;
        break;

      case 'x': case 'X':

        if (interim_command == robot_control_mode_t::standby or interim_status == robot_control_mode_t::standby)
        {   swap_state = true;
          interim_status = robot_control_mode_t::quit;
          std::cout << yellow_key << "Ending program - no more inputs..." << color_key << std::endl;
        }
        else {
          std::cout << yellow_key << "Not ready to shutdown" << color_key << std::endl;

        }
        break;

      case '2': case '3': case '4': case '5': case '6': case '7': case '8': case '9': case '0':
        std::cout << green_key << "Quick-swapped to new mode!" << color_key << std::endl;
        interim_status = static_cast<robot_control_mode_t>(std::stoi(&c));
        break;

      case '1':

        c = cin.get();

        switch(c){
        // If 1+Enter typed
        case '\n':
          std::cout << green_key << "Quick-swapped to new mode!" << color_key << std::endl;
          interim_status = robot_control_mode_t::standby;
          break;

        // If 1+0+x typed
        case '0':
          c = cin.get();

          switch(c){

          // If 107 typed
          case '5': case '6': case '7': case '8': case '9':
            std::cout << green_key << "Quick-swapped to new mode!" << color_key << std::endl;
            interim_status = static_cast<robot_control_mode_t>(100+stoi(&c));
            break;
          default:
            ROS_ERROR("Unrecognized Command");
            break;
          }
          break;
        default:
          ROS_ERROR("Unrecognized Command");
          break;
        }
        break;
      case 's':
        std::cout << green_key << "Quick-swapped to new mode!" << color_key << std::endl;
        interim_status = robot_control_mode_t::shutdown_control;
        break;
      case ' ':
        std::cout << yellow_key << robot_control_labels[interim_status] << color_key << " is the state currently active" << std::endl << std::endl;
        break;
      default:
        ROS_ERROR("Unrecognized Command");
        break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

  }
  else
  {
    std::cout << yellow_key << robot_control_labels[interim_status] << color_key << " is the state currently active" << std::endl << std::endl;
  } // else

} // while

}


/***************************/
/* ROS Subscriber Callback */
/***************************/

void esmacat_ros_interface_class::ROS_subscribe_callback(const agree_esmacat_pkg::agree_esmacat_command msg)
{
  // Save data from ROS message to shared memory
  interim_command                            = (robot_control_mode_t) msg.command;

  //Check if command is updated by ROS commands.
  if(prev_command != interim_command)  {
    ROS_INFO("Change MODE to: %d",interim_command);
  }

  //Check if damping or stiffness values are updated by ROS commands.
  bool damping_updated = false;
  bool stiffness_updated = false;
  for(unsigned long joint_index = 0; joint_index < 5; joint_index++){
    if(abs(prev_damping[joint_index] - msg.damping_d[joint_index])>1E-3) damping_updated = true;
    if(abs(prev_stiffness[joint_index] - msg.stiffness_k[joint_index])>1E-3) stiffness_updated = true;
  }

  // Check if weight assistance is updated by ROS commands.
  bool weight_assistance_updated = false;
  for(unsigned long segment_index = 0; segment_index < 2 ; segment_index++){
    if(abs(prev_weight_assistance[segment_index] - msg.weight_assistance[segment_index])>1E-3) weight_assistance_updated = true;
  }

  // If damping, stiffness or weight assistance values are updated raise ROS info.
  if(damping_updated){
    ROS_INFO("Update DAMPING to: %.2f %.2f %.2f %.2f %.2f ",msg.damping_d[0],msg.damping_d[1],msg.damping_d[2],msg.damping_d[3],msg.damping_d[4]);
  }
  if(stiffness_updated){
    ROS_INFO("Update STIFFNESS to: %.2f %.2f %.2f %.2f %.2f ",msg.stiffness_k[0],msg.stiffness_k[1],msg.stiffness_k[2],msg.stiffness_k[3],msg.stiffness_k[4]);
  }
  if(weight_assistance_updated){
    ROS_INFO("Update WEIGHT ASSISTANCE to: %.2f %.2f",msg.weight_assistance[0],msg.weight_assistance[1]);
  }

  // Update previous values to be compared in next cycle
  prev_command                  = msg.command;
  prev_weight_assistance[0]     = msg.weight_assistance[0];
  prev_weight_assistance[1]     = msg.weight_assistance[1];
  for(unsigned long joint_index = 0; joint_index < 5; joint_index++){
    prev_stiffness[joint_index] = msg.stiffness_k[joint_index];
    prev_damping[joint_index]   = msg.damping_d[joint_index];
  }
}


/***************************/
/* ROS Parameters Callback */
/***************************/
/**
 * @brief esmacat_ros_interface_class::ROS_parameters_callback
 * @param event
 * This callback function updates the shared-memory variables with ROS parameters
 */
void esmacat_ros_interface_class::ROS_parameters_callback(__attribute__((unused)) const ros::TimerEvent& event){

  // ROS handler
  ros::NodeHandle nh;

  // Declar ROM vectors
  std::vector<float> ROM_max,ROM_min;

  float weight,height,length_forearm,length_upperarm,length_hand;
  int length_robot_upperarm = 0;
  int side;


  // TODO: There should be a check if the ROMs are compatible with the side...
  if (nh.hasParam("physiological_param"))
  {
    // Get ROS parameters for ROM maximum values (motor reference)
    if( !nh.getParam("/physiological_param/ROM_Max", ROM_max) ){
      ROS_ERROR("Failed to get ROM Max parameters from server.");
    }

    // Get ROS parameters for ROM minimum values (motor reference)
    if( !nh.getParam("/physiological_param/ROM_Min", ROM_min) ){
      ROS_ERROR("Failed to get ROM Min parameters from server.");
    }

  }
  else
  {
    ROS_ERROR("Failed to get ROS parameters 'physiological_param'");
  }

  if (nh.hasParam("physiological_param"))
  {
    nh.getParam("physiological_param/weight",weight );
    nh.getParam("physiological_param/height",height );
    nh.getParam("physiological_param/length_forearm",length_forearm );
    nh.getParam("physiological_param/length_upperarm",length_upperarm );
    nh.getParam("physiological_param/length_hand",length_hand );
  }
  else
  {
    ROS_ERROR("Failed to get ROS parameters 'physiological_param'");
  }

  if (nh.hasParam("robot_param"))
  {
    nh.getParam("robot_param/length_robot_upperarm",length_robot_upperarm );
  }
  else
  {
    ROS_ERROR("Failed to get ROS parameters 'robot_param'");
  }

  if (nh.hasParam("side"))
  {
    nh.getParam("side",side );
  }
  else
  {
    ROS_ERROR("Failed to get ROS parameters 'side'");
  }

}
