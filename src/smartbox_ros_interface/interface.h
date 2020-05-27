#ifndef INTERFACE_H
#define INTERFACE_H

#include <iostream>
#include <thread>
#include <chrono>
#include "ros/ros.h"
#include "std_msgs/Int64.h"
//#include "esmacat_pkg/esmacat_sensor.h"
//#include "esmacat_pkg/esmacat_command.h"
#include <boost/thread/thread.hpp>

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

enum RobotState
{
  EXIT,
  STOP,
  CURRENT,
  TORQUE,
  NULLTORQUE,
  GRAVITY,
  FREEZE,
  QUIT,
};

class smartbox_interface
{
public:
  smartbox_interface()
  {
    boost_ROS_publish_thread    = boost::thread(&smartbox_interface::ROS_publish_thread, this);
    //boost_ROS_subscribe_thread  = boost::thread(&smartbox_interface::ROS_subscribe_thread, this);
    boost_ROS_command_thread  = boost::thread(&smartbox_interface::ROS_command_thread, this);
    std::cout << "ROS interface objects instantiated" << std::endl;

  }

  ~smartbox_interface()
  {
    std::cout << "ROS interface threads joining" << std::endl;
    boost_ROS_publish_thread.join();
//    boost_ROS_subscribe_thread.join();
  }

  uint64_t interim_state;

private:

  boost::thread boost_ROS_publish_thread;
//  boost::thread boost_ROS_subscribe_thread;
  boost::thread boost_ROS_command_thread;

//  void ROS_subscribe_thread();
  void ROS_publish_thread();
  void ROS_command_thread();
//  void ROS_subscribe_callback(const esmacat_pkg::esmacat_sensor msg);

  void print_command_keys();


};


#endif // INTERFACE_H
