#include "ros_interface.h"
#include "ros/ros.h"

/************************/
/* ROS Publisher Thread */
/************************/

void esmacat_ros_interface::ROS_publish_thread(){


  //Declare a message and setup the publisher for that message
//  esmacat_ros_interface::esmacat_command command;
  std_msgs::Int64 msg;
  ros::NodeHandle n;
  ros::Rate loop_rate(100);
  ros::Publisher publisher = n.advertise<std_msgs::Int64>("esmacat_sensor", 1000);


  //Variables that setup the publishing loop
  int interim_roscount = 0;

  while (ros::ok()){

//    command.setpoint = (int64_t) 100*sin((2.0*3.14159)*interim_roscount/100.0);
//    command.state = interim_state;
      msg.data = esmacat_sm.data->loop_cnt;
      publisher.publish(msg);

//    if(esmacat_sm.data->stop == true) ros::shutdown();

    loop_rate.sleep();
    interim_roscount++;
  }

}

/************************/
/* ROS Subscriber Thread */
/************************/

void esmacat_ros_interface::ROS_subscribe_thread(){

  //Setup a subscriber that will get data from other ROS nodes
  ros::MultiThreadedSpinner spinner(1); // Use 4 threads

  ros::NodeHandle n;

  ros::Subscriber subscriber = n.subscribe("esmacat_command", 1000, &esmacat_ros_interface::ROS_subscribe_callback, this);

  spinner.spin();
}

//void esmacat_ros_interface::ROS_command_thread(){

//  //Initialize Robot status
//  char c;
//  string inputString;
//  RobotState state(STOP);
//  bool swap_state(false);

//  print_command_keys();

//  while (ros::ok()){


//    // Get character
//    c = cin.get();

//    if(c != '\n'){
//      switch(c){

//      case 's': case 'S':
//        if (state != STOP)
//        {
//          std::cout << green_key << "Quick-swapped to STOP mode!" << color_key << std::endl;
//          state = STOP;
//          swap_state = true;
//        }
//        else
//        {
//          std::cout << yellow_key << "Already in STOP mode" << color_key <<  std::endl;
//        }
//        break;
//      case 'n': case 'N':
//        if (state != NULLTORQUE)
//        {
//          std::cout << green_key << "Quick-swapped to NULL-TORQUE mode!" << color_key << std::endl;
//          state = NULLTORQUE;
//          swap_state = true;
//        }
//        else
//        {
//          std::cout << yellow_key << "Already in NULL-TORQUE mode" << color_key <<  std::endl;
//        }
//        break;
//      case 'c': case 'C':
//        if (state != CURRENT)
//        {
//          std::cout << green_key << "Quick-swapped to CURRENT mode!" << color_key << std::endl;
//          state = CURRENT;
//          swap_state = true;
//        }
//        else
//        {
//          std::cout << yellow_key << "Already in CURRENT mode" << color_key <<  std::endl;
//        }
//        break;
//      case 't': case 'T':
//        if (state != TORQUE)
//        {
//          std::cout << green_key << "Quick-swapped to TORQUE mode!" << color_key << std::endl;
//          state = TORQUE;
//          swap_state = true;
//        }
//        else
//        {
//          std::cout << yellow_key << "Already in NULL-TORQUE mode" << color_key <<  std::endl;
//        }
//        break;

//      case 'g': case 'G':
//        if (state != GRAVITY)
//        {
//          std::cout << green_key << "Quick-swapped to GRAVITY mode!" << color_key << std::endl;
//          state = GRAVITY;
//          swap_state = true;
//        }
//        else
//        {
//          std::cout << yellow_key << "Already in GRAVITY mode" << color_key <<  std::endl;
//        }
//        break;
//      case 'f': case 'F':
//        if (state != FREEZE)
//        {
//          std::cout << green_key << "Quick-swapped to FREEZE mode!" << color_key << std::endl;
//          state = FREEZE;
//          swap_state = true;
//        }
//        else
//        {
//          std::cout << yellow_key << "Already in FREEZE mode" << color_key <<  std::endl;
//        }
//        break;
//      case 'k': case 'K':

//        if (state == STOP or state == EXIT)
//        {   swap_state = true;
//          state = EXIT;
//          std::cout << yellow_key << "Ending program - no more inputs..." << color_key << std::endl;
//        }
//        else {
//          std::cout << yellow_key << "First stop the motor" << color_key << std::endl;

//        }
//        break;
//      case ' ':
//        print_command_keys();
//        break;
//      default:
//        std::cout << red_key << "Unrecognized key input!" << color_key <<  std::endl;
//        break;
//      }

//      /**
//                 * This is a message object. You stuff it with data, and then publish it.
//                 */


//      interim_state = state;

//      std::this_thread::sleep_for(std::chrono::milliseconds(100));

//    }
//    else
//    {
//      std::cout << yellow_key << state_labels[state] << color_key << " is the state currently active" << std::endl;
//    } // else

//  } // while

//}

void esmacat_ros_interface::ROS_subscribe_callback(const std_msgs::Int64 msg)
{
  //Display data from hard real-time loop to the the terminal.
  //  ROS_INFO(" Enc:[%i]",msg.data);
  esmacat_sm.data->elapsed_time =  msg.data;
  std::cout << esmacat_sm.data->elapsed_time << endl;
}

// Print commands on terminal
void esmacat_ros_interface::print_command_keys()
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
