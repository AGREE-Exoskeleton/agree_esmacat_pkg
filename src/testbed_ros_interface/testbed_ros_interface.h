#ifndef INTERFACE_H
#define INTERFACE_H

#include <iostream>
#include <thread>
#include <chrono>
#include <boost/thread/thread.hpp>
#include <time.h>

#include <math.h>
#define BILLION 1000000000L

#include "ros/ros.h"

#include "agree_esmacat_pkg/agree_esmacat_status.h"
#include "agree_esmacat_pkg/agree_esmacat_command.h"

#include "std_msgs/Int64.h"

#define EXIT    0
#define STOP    1
#define CURRENT 2
#define TORQUE  3
#define NULLTORQUE 4
#define GRAVITY 5
#define FREEZE  6
#define IMPEDANCE 7
#define HOMING  8
#define POSITION 9

#define SUB_TASK_DURATION 4000

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
  "IMPEDANCE",
    "HOMING",
    "POSITION",
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
//  IMPEDANCE,
//};

class testbed_ros_interface
{
public:
  testbed_ros_interface()
  {
    boost_ROS_publish_thread    = boost::thread(&testbed_ros_interface::ROS_publish_thread, this);
    boost_ROS_subscribe_thread  = boost::thread(&testbed_ros_interface::ROS_subscribe_thread, this);
    boost_ROS_command_thread  = boost::thread(&testbed_ros_interface::ROS_command_thread, this);
    boost_adaptive_control_thread  = boost::thread(&testbed_ros_interface::adaptive_control_thread, this);

    std::cout << "ROS interface objects instantiated" << std::endl;
    interim_state = STOP;
    interim_impedance_stiffness = 0;
    interim_impedance_damping = 0;
    interim_setpoint = 0;
    interim_duration = SUB_TASK_DURATION; // ms
    interim_elapsed_time = 0;
    interim_sign = 1;
    interim_timestamp = 0;
  }

  ~testbed_ros_interface()
  {
    std::cout << "ROS interface threads joining" << std::endl;
    boost_ROS_publish_thread.join();
    //boost_ROS_command_thread.join();
    boost_ROS_subscribe_thread.join();
    boost_adaptive_control_thread.join();

  }

  // Command variables
  uint64_t interim_state;
  bool interim_swap_state = false;

  float interim_impedance_damping;
  float interim_impedance_stiffness;
  float interim_setpoint;
  float interim_duration; // Sub-task duration
  float interim_setpoint_final;

  // Status variables
  float interim_position;
  float interim_speed;
  float interim_torque;
  float interim_elapsed_time;
  float interim_timestamp;

  // Control variables
  float interim_score_t;
  float interim_score_k;
  float interim_elapsed_time_offset;
  float interim_position_offset;
  int interim_sign;



private:

  boost::thread boost_ROS_publish_thread;
  boost::thread boost_ROS_subscribe_thread;
  boost::thread boost_ROS_command_thread;
  boost::thread boost_adaptive_control_thread;

  void ROS_subscribe_thread();
  void ROS_publish_thread();
  void ROS_command_thread();
  void adaptive_control_thread();
  void ROS_subscribe_callback(const agree_esmacat_pkg::agree_esmacat_status msg);

  void print_command_keys();

  timespec diff(timespec , timespec );

};


#endif // INTERFACE_H
