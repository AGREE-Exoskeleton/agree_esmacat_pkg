#ifndef INTERFACE_H
#define INTERFACE_H

#include <iostream>
#include <thread>
#include <chrono>
#include <boost/thread/thread.hpp>

#include "ros/ros.h"
#include "agree_esmacat_pkg/agree_esmacat_status.h"
#include "agree_esmacat_pkg/agree_esmacat_command.h"

#include "std_msgs/Int64.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"

#include "esmacat_applications/agree_esmacat_rt/agree_joint_structs.h"


using namespace std;

// Text Color Identifiers
const string boldred_key = "\033[1;31m";
const string red_key = "\033[31m";
const string boldpurple_key = "\033[1;35m";
const string yellow_key = "\033[33m";
const string blue_key = "\033[36m";
const string green_key = "\033[32m";
const string color_key = "\033[0m";

class esmacat_ros_interface_class
{
public:
    esmacat_ros_interface_class()
    {
        interim_command = robot_control_mode_t::standby;
        interim_status = robot_control_mode_t::standby;
        // Start Boost thread
        boost_ROS_publish_thread    = boost::thread(&esmacat_ros_interface_class::ROS_publish_thread, this);
        boost_ROS_command_thread    = boost::thread(&esmacat_ros_interface_class::ROS_command_thread, this);

        ROS_INFO("AGREE Simulation publish thread instantiated");
    }

  ~esmacat_ros_interface_class()
  {
    boost_ROS_publish_thread.join();
  }

private:

  robot_control_mode_t interim_command;
  robot_control_mode_t interim_status;

  uint16_t prev_command;
  double   prev_stiffness[5];
  double   prev_damping[5];
  double   prev_weight_assistance[2];

  boost::thread boost_ROS_publish_thread;
  boost::thread boost_ROS_command_thread;


  void ROS_publish_thread();
  void ROS_command_thread();
  void ROS_parameters_thread();

public:
  void ROS_parameters_callback(const ros::TimerEvent& event);
  void ROS_subscribe_callback(const agree_esmacat_pkg::agree_esmacat_command msg);

};


#endif // INTERFACE_H
