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
#include "agree_shared_memory_comm.h"


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
        // Initializing the shared memory
        if (esmacat_sm.init())
        {
            int key = esmacat_sm.get_shared_memory_key();
            ROS_INFO("AGREE SHM Interface shared memory initialized with key 0x%x", key);    // start the shared memory communication
        }
        else
        {
            cout << "AGREE SHM Interface shared memory initialization has been failed";
            esmacat_sm.detach_shared_memory();
        }

        // Initialize command and status
        esmacat_sm.set_esmacat_command(robot_control_mode_t::standby);
        esmacat_sm.set_esmacat_status(robot_control_mode_t::standby);
        esmacat_sm.set_use_ros(true);

        // Start Boost thread
        boost_ROS_publish_thread    = boost::thread(&esmacat_ros_interface_class::ROS_publish_thread, this);

        ROS_INFO("AGREE SHM Interface threads instantiated");
    }

  ~esmacat_ros_interface_class()
  {
    boost_ROS_publish_thread.join();
  }

  agree_shared_memory_comm esmacat_sm;

private:

  uint16_t prev_command;
  double   prev_stiffness[5];
  double   prev_damping[5];
  double   prev_weight_assistance[2];

  boost::thread boost_ROS_publish_thread;

  void ROS_subscribe_thread();
  void ROS_publish_thread();
  void ROS_parameters_thread();

public:
  void ROS_parameters_callback(const ros::TimerEvent& event);
  void ROS_subscribe_callback(const agree_esmacat_pkg::agree_esmacat_command msg);

};


#endif // INTERFACE_H
