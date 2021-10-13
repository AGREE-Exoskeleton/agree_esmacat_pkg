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

  //    Inform the real-time thread the system is running through ROS
  ROS_WARN("ROS enabled.");

  while (ros::ok()){

    esmacat_sm.set_use_ros(true);

    msg.elapsed_time    = esmacat_sm.data->elapsed_time_ms;
    msg.status          = esmacat_sm.data->control_mode_status;

    msg.joint_position_rad.clear();
    msg.joint_speed_rad_s.clear();
    msg.joint_torque_mNm.clear();
    msg.setpoint_torque_mNm.clear();
    msg.setpoint_position_rad.clear();


    for(int joint_index=0;joint_index<5;joint_index++){
      msg.joint_position_rad.push_back(esmacat_sm.data->joint_values[joint_index].incremental_encoder_reading_radians);
      msg.joint_speed_rad_s.push_back(esmacat_sm.data->joint_values[joint_index].incremental_encoder_speed_radians_sec);
      msg.joint_torque_mNm.push_back(esmacat_sm.data->joint_values[joint_index].filtered_load_mNm);
      msg.setpoint_torque_mNm.push_back(esmacat_sm.data->impedance_control_terms[joint_index].torque_setpoint_mNm);
      msg.setpoint_position_rad.push_back(esmacat_sm.data->impedance_control_terms[joint_index].impedance_control_setpoint_rad);
    }

    publisher.publish(msg);

    loop_rate.sleep();
    interim_roscount++;



    if (esmacat_sm.data->control_mode_command == 0 || esmacat_sm.data->control_mode_status == 0 || ros::master::check() == 0)
    {
      esmacat_sm.set_use_ros(false);
      ROS_WARN("ROS disabled.");
      ROS_INFO("AGREE SHM Interface exit conditions met and shutting down..");
      esmacat_sm.detach_shared_memory();
      ros::shutdown();
      break;
    }
  }

}

/***************************/
/* ROS Subscriber Callback */
/***************************/

void esmacat_ros_interface_class::ROS_subscribe_callback(const agree_esmacat_pkg::agree_esmacat_command msg)
{
  // Save data from ROS message to shared memory
  esmacat_sm.data->control_mode_command                             = (robot_control_mode_t) msg.command;

  // Save weight assistance gains to shared memory
  esmacat_sm.data->arm_weight_compensation_config.upperarm_weight_assistance = msg.weight_assistance[0];
  esmacat_sm.data->arm_weight_compensation_config.forearm_weight_assistance = msg.weight_assistance[1];


  for(unsigned long joint_index = 0; joint_index < 5; joint_index++){
    esmacat_sm.data->impedance_control_command[joint_index].impedance_control_k_gain_mNm_per_rad         = msg.stiffness_k[joint_index];
    esmacat_sm.data->impedance_control_command[joint_index].impedance_control_d_gain_mNm_per_rad_per_sec = msg.damping_d[joint_index];
    esmacat_sm.data->impedance_control_command[joint_index].impedance_control_setpoint_rad               = msg.setpoint[joint_index];
  }

  //Check if command is updated by ROS commands.
  if(prev_command != msg.command)  {
    ROS_INFO("Change MODE to: %d",msg.command);
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
    if( !nh.getParam("/physiological_param/ROM_Max", ROM_max) )
      ROS_ERROR("Failed to get ROM Max parameters from server.");
    else{
      cout << "ROM Max: ";
      for (auto i: ROM_max)
        std::cout << i << ' ';
      cout << endl;
    }

    // Get ROS parameters for ROM minimum values (motor reference)
    if( !nh.getParam("/physiological_param/ROM_Min", ROM_min) )
      ROS_ERROR("Failed to get ROM Min parameters from server.");
    else{
      cout << "ROM Min: ";
      for (auto i: ROM_min)
        std::cout << i << ' ';
      cout << endl;
    }
  }
  else
  {
    ROS_ERROR("Failed to get ROS parameters 'physiological_param'");
  }


  // NOTE: J2 override
  ROM_max[1] = 0.0/180.0*M_PI;

  for (unsigned long joint_index=0;joint_index<5;joint_index++){
    // TODO: Check conversion to robot min/max...
    esmacat_sm.data->impedance_control_command[joint_index].soft_stop_lower_limit_rad = ROM_min[joint_index];
    esmacat_sm.data->impedance_control_command[joint_index].soft_stop_upper_limit_rad = ROM_max[joint_index];
  }



  if (nh.hasParam("physiological_param"))
  {
    nh.getParam("physiological_param/weight",weight );
    nh.getParam("physiological_param/height",height );
    nh.getParam("physiological_param/length_forearm",length_forearm );
    nh.getParam("physiological_param/length_upperarm",length_upperarm );
    nh.getParam("physiological_param/length_hand",length_hand );

    esmacat_sm.data->arm_weight_compensation_config.human_weight_kg  =   weight;
    esmacat_sm.data->arm_weight_compensation_config.human_height_m   =   height;
    esmacat_sm.data->arm_weight_compensation_config.forearm_length_m =   length_forearm;
    esmacat_sm.data->arm_weight_compensation_config.upperarm_length_m =  length_upperarm;
    esmacat_sm.data->arm_weight_compensation_config.hand_length_m     =  length_hand;
  }
  else
  {
    ROS_ERROR("Failed to get ROS parameters 'physiological_param'");
  }

  if (nh.hasParam("robot_param"))
  {
    nh.getParam("robot_param/length_robot_upperarm",length_robot_upperarm );

    // Create mapping from 1-2-3-4-5 tags and robot lengths
    switch(length_robot_upperarm){

    case 1:
      esmacat_sm.data->arm_weight_compensation_config.length_robot_upperarm_m = 0.28;
      break;

    case 2:
      esmacat_sm.data->arm_weight_compensation_config.length_robot_upperarm_m = 0.2925;
      break;

    case 3:
      esmacat_sm.data->arm_weight_compensation_config.length_robot_upperarm_m = 0.305;
      break;

    case 4:
      esmacat_sm.data->arm_weight_compensation_config.length_robot_upperarm_m = 0.3175;
      break;

    default:
      esmacat_sm.data->arm_weight_compensation_config.length_robot_upperarm_m = 0.28;
      break;

    }
  }
  else
  {
    ROS_ERROR("Failed to get ROS parameters 'robot_param'");
  }

  if (nh.hasParam("side"))
  {
    nh.getParam("side",side );
    esmacat_sm.data->arm_weight_compensation_config.side = side;
  }
  else
  {
    ROS_ERROR("Failed to get ROS parameters 'side'");
  }

}
