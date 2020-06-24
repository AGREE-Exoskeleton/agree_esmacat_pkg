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
#include "std_msgs/String.h"

#include "esmacat_shared_memory_comm.h"

using namespace std;

// Text Color Identifiers
const string boldred_key = "\033[1;31m";
const string red_key = "\033[31m";
const string boldpurple_key = "\033[1;35m";
const string yellow_key = "\033[33m";
const string blue_key = "\033[36m";
const string green_key = "\033[32m";
const string color_key = "\033[0m";

//Labels for states
const string state_labels[] = {
  "EXIT",
  "STOP",
  "CURRENT",
  "TORQUE",
  "NULLTORQUE",
  "GRAVITY",
  "FREEZE",
  "QUIT",
};

//enum RobotState
//{
//  EXIT,
//  STOP,
//  CURRENT,
//  TORQUE,
//  NULLTORQUE,
//  GRAVITY,
//  FREEZE,
//  QUIT,
//};

class esmacat_ros_interface_class
{
public:
  esmacat_ros_interface_class()
  {
    boost_ROS_publish_thread    = boost::thread(&esmacat_ros_interface_class::ROS_publish_thread, this);
    boost_ROS_subscribe_thread  = boost::thread(&esmacat_ros_interface_class::ROS_subscribe_thread, this);
//    boost_ROS_command_thread  = boost::thread(&esmacat_ros_interface::ROS_command_thread, this);
    ROS_INFO("ROS threads instantiated");
    esmacat_sm.init();
  }

  ~esmacat_ros_interface_class()
  {
    std::cout << "ROS interface threads joining" << std::endl;
    boost_ROS_publish_thread.join();
    boost_ROS_subscribe_thread.join();
  }

//  RobotState interim_state;elapsed_time
  esmacat_shared_memory_comm esmacat_sm;

private:

  uint64_t prev_state;

  boost::thread boost_ROS_publish_thread;
  boost::thread boost_ROS_subscribe_thread;
//  boost::thread boost_ROS_command_thread;

  void ROS_subscribe_thread();
  void ROS_publish_thread();
//  void ROS_command_thread();
  void ROS_subscribe_callback(const agree_esmacat_pkg::agree_esmacat_command msg);

  void print_command_keys();


};


#endif // INTERFACE_H
