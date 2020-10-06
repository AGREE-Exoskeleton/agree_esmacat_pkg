#ifndef INTERFACE_H
#define INTERFACE_H

#include <iostream>
#include <fstream>
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

// EsmaBox FSM
#define EXIT        0
#define STOP        1
#define CURRENT     2
#define TORQUE      3
#define NULLTORQUE  4
#define GRAVITY     5
#define FREEZE      6
#define IMPEDANCE   7
#define HOMING      8
#define POSITION    9
#define WEIGHT      10
#define IMPEDANCE_EXT 11
#define TRIGGER     12
#define ADAPTIVE    13
#define PASSIVE     14
#define RESISTIVE   15
#define CHALLENGING 16

#define HOMING_DONE 108
#define POSITION_DONE 109

// Exercise FSM
#define REST        0
#define WAIT        1
#define MOVE_UP     2
#define MOVE_DOWN   3

#define EXERCISE_START              -M_PI/2.0
#define EXERCISE_STOP               M_PI/2
#define EXERCISE_DURATION           8000.0
#define EXERCISE_AMPLITUDE          M_PI/2.0

#define STIFFNESS_TO_DAMPING_RATIO  0.20

#define TRIGGER_TORQUE_THRESHOLD    750
#define TRIGGER_THRESHOLD           0.1
#define TRIGGER_REST_TIMEOUT        200 // 100 counter = 1s
#define TRIGGER_TIMEOUT             400
#define TRIGGER_POSITION            0
#define TRIGGER_TORQUE              1

#define SUB_TASK_DURATION           4000

#define N_DOFS_MAX                  5

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
    "WEIGHT",
    "IMPEDANCE_EXT",
    "TRIGGER",
    "ADAPTIVE",
    "PASSIVE",
    "RESISTIVE",
    "CHALLENGING",
};

class testbed_ros_interface
{
public:
  testbed_ros_interface()
  {
    boost_ROS_publish_thread    = boost::thread(&testbed_ros_interface::ROS_publish_thread, this);
    boost_ROS_subscribe_thread  = boost::thread(&testbed_ros_interface::ROS_subscribe_thread, this);
    boost_ROS_command_thread  = boost::thread(&testbed_ros_interface::ROS_command_thread, this);
    boost_adaptive_control_thread  = boost::thread(&testbed_ros_interface::control_thread, this);

    std::cout << "ROS interface objects instantiated" << std::endl;
    interim_command                  = STOP;
    interim_impedance_stiffness     = 0;
    interim_impedance_damping       = 0;
    saved_impedance_stiffness       = 10.0;
    saved_impedance_damping         = 1.0;

    interim_weight_assistance       = 1.0;

    interim_setpoint                = 0;
    interim_position                = 0;
    interim_setpoint_start          = 0;
    interim_setpoint_final          = 0;
    interim_duration                = SUB_TASK_DURATION; // ms
    interim_elapsed_time            = 0;
    interim_sign                    = 1;
    interim_timestamp               = 0;
    trigger_mode                    = TRIGGER_POSITION;


    adaptive_impedance_stiffness    = 0.0;
    adaptive_filtered_error_rad         = 0.0;
    adaptive_gain                   = 1.0;
    adaptive_forgetting_factor      = 0.95;
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
  uint64_t interim_command;
  uint64_t interim_exercise_status = REST;
  float    interim_exercise_counter = 0;

  bool      interim_swap_state;

  // Impedance Parameters
  float interim_impedance_damping;
  float saved_impedance_damping;

  float interim_impedance_stiffness;
  float saved_impedance_stiffness;

  float interim_weight_assistance;

  float interim_setpoint = 0;
  float interim_duration; // Sub-task duration
  float interim_amplitude;
  float interim_setpoint_final;
  float interim_setpoint_start;
  int trigger_mode;

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

  // Adaptive Control
  float adaptive_filtered_error_rad;
  float adaptive_impedance_stiffness;
  float adaptive_impedance_damping;
  float adaptive_gain;
  float adaptive_forgetting_factor;

  int interim_sign;

  // Log
  ofstream log;
  void openfile();
  void write2file();
  void closefile();

private:

  boost::thread boost_ROS_publish_thread;
  boost::thread boost_ROS_subscribe_thread;
  boost::thread boost_ROS_command_thread;
  boost::thread boost_adaptive_control_thread;

  void ROS_subscribe_thread();
  void ROS_publish_thread();
  void ROS_command_thread();
  void control_thread();
  void ROS_subscribe_callback(const agree_esmacat_pkg::agree_esmacat_status msg);

  void print_command_keys();

  timespec diff(timespec , timespec );

};


#endif // INTERFACE_H
