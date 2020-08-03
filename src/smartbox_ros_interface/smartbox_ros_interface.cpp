#include "smartbox_ros_interface.h"

void smartbox_interface::ROS_publish_thread(){

  struct timespec t0,temp,timestamp;
  clock_gettime( CLOCK_REALTIME, &t0);

  //Declare a message and setup the publisher for that message
  ros::NodeHandle n;
  ros::Rate loop_rate(100);
  agree_esmacat_pkg::agree_esmacat_command msg;

  ros::Publisher pub_esmacat_write = n.advertise<agree_esmacat_pkg::agree_esmacat_command>("esmacat/command",1000);

  //Variables that setup the publishing loop
  int interim_roscount = 0;
  double sine = 0;

  while (ros::ok()){

    msg.mode     = interim_state;
    msg.damping_d = interim_impedance_damping;
    msg.stiffness_k = interim_impedance_stiffness;
    sine = sin(2*3.1415*(double)interim_roscount++/500.0);
    msg.setpoint = M_PI/4*sine;

    clock_gettime( CLOCK_REALTIME, &temp);

    timestamp = diff(t0,temp);
    msg.timestamp = (float)(timestamp.tv_sec);

    pub_esmacat_write.publish(msg);
    if(interim_state==0) ros::shutdown();

    loop_rate.sleep();
  }

}

/************************/
/* ROS Subscriber Thread */
/************************/


void smartbox_interface::ROS_subscribe_thread(){

  //Setup a subscriber that will get data from other ROS nodes
  ros::MultiThreadedSpinner spinner(1); // Use 4 threads

  ros::NodeHandle n;

  ros::Subscriber subscriber = n.subscribe("esmacat/status", 1000, &smartbox_interface::ROS_subscribe_callback, this);

  spinner.spin();
}


void smartbox_interface::ROS_command_thread(){

  //Initialize Robot status
  char c;
  string inputString;
  uint8_t state(STOP);

  bool swap_state(false);

  print_command_keys();

  while (ros::ok()){


    // Get character
    c = cin.get();

    if(c != '\n'){
      switch(c){

      case 's': case 'S':
        if (state != STOP)
        {
          std::cout << green_key << "Quick-swapped to STOP mode!" << color_key << std::endl;
          state = STOP;
          swap_state = true;
        }
        else
        {
          std::cout << yellow_key << "Already in STOP mode" << color_key <<  std::endl;
        }
        break;
      case 'n': case 'N':
        if (state != NULLTORQUE)
        {
          std::cout << green_key << "Quick-swapped to NULL-TORQUE mode!" << color_key << std::endl;
          state = NULLTORQUE;
          swap_state = true;
        }
        else
        {
          std::cout << yellow_key << "Already in NULL-TORQUE mode" << color_key <<  std::endl;
        }
        break;
      case 'c': case 'C':
        if (state != CURRENT)
        {
          std::cout << green_key << "Quick-swapped to CURRENT mode!" << color_key << std::endl;
          state = CURRENT;
          swap_state = true;
        }
        else
        {
          std::cout << yellow_key << "Already in CURRENT mode" << color_key <<  std::endl;
        }
        break;
      case 't': case 'T':
        if (state != TORQUE)
        {
          std::cout << green_key << "Quick-swapped to TORQUE mode!" << color_key << std::endl;
          state = TORQUE;
          swap_state = true;
        }
        else
        {
          std::cout << yellow_key << "Already in NULL-TORQUE mode" << color_key <<  std::endl;
        }
        break;

      case 'g': case 'G':
        if (state != GRAVITY)
        {
          std::cout << green_key << "Quick-swapped to GRAVITY mode!" << color_key << std::endl;
          state = GRAVITY;
          swap_state = true;
        }
        else
        {
          std::cout << yellow_key << "Already in GRAVITY mode" << color_key <<  std::endl;
        }
        break;
      case 'f': case 'F':
        if (state != FREEZE)
        {
          std::cout << green_key << "Quick-swapped to FREEZE mode!" << color_key << std::endl;
          state = FREEZE;
          swap_state = true;
        }
        else
        {
          std::cout << yellow_key << "Already in FREEZE mode" << color_key <<  std::endl;
        }
        break;
      case 'i': case 'I':
        if (state != IMPEDANCE)
        {
          std::cout << green_key << "Quick-swapped to IMPEDANCE mode!" << color_key << std::endl;
          state = IMPEDANCE;
          swap_state = true;
        }
        else
        {
          std::cout << yellow_key << "Already in FREEZE mode" << color_key <<  std::endl;
        }
        break;

      case 'p': case 'P':
        if (state != POSITION)
        {
          std::cout << green_key << "Quick-swapped to POSITION mode!" << color_key << std::endl;
          state = POSITION;
          swap_state = true;
        }
        else
        {
          std::cout << yellow_key << "Already in POSITION mode" << color_key <<  std::endl;
        }
        break;

      case 'h': case 'H':
        if (state != HOMING)
        {
          std::cout << green_key << "Quick-swapped to HOMING mode!" << color_key << std::endl;
          state = HOMING;
          swap_state = true;
        }
        else
        {
          std::cout << yellow_key << "Already in HOMING mode" << color_key <<  std::endl;
        }
        break;

      case 'x': case 'X':

        if (state == STOP or state == EXIT)
        {   swap_state = true;
          state = EXIT;
          std::cout << yellow_key << "Ending program - no more inputs..." << color_key << std::endl;
        }
        else {
          std::cout << yellow_key << "First stop the motor" << color_key << std::endl;

        }
        break;
      case 'd': case 'D':
        if(c != '\n'){
          std::cout << yellow_key << "Impedance DAMPING change selection:" << color_key << std::endl;
          c = cin.get();
          switch(c){
          case '+':

            interim_impedance_damping += 0.5;
            std::cout << green_key << "Impedance DAMPING increased to " << yellow_key << interim_impedance_damping <<  color_key << std::endl;

            break;
          case '-':
            if(interim_impedance_damping > 0)
            {
              interim_impedance_damping -= 0.5;
              std::cout << green_key << "Impedance DAMPING decreased to " << yellow_key << interim_impedance_damping <<  color_key << std::endl;
            }
            else
            {
              ROS_WARN("Impedance DAMPING not changed: %f", interim_impedance_damping);
            }
            break;
          default:
            ROS_WARN("Impedance DAMPING not changed: %f", interim_impedance_damping);

            break;
          }
        }
        break;

      case 'k': case 'K':
        if(c != '\n'){
          std::cout << yellow_key << "Impedance STIFFNESS change selection: " << color_key << std::endl;
          c = cin.get();
          switch(c){
          case '+':

            interim_impedance_stiffness += 0.5;
            std::cout << green_key << "Impedance STIFFNESS increased to " << yellow_key << interim_impedance_stiffness <<  color_key << std::endl;

            break;
          case '-':
            if(interim_impedance_stiffness > 0)
            {
              interim_impedance_stiffness -= 0.5;
              std::cout << green_key << "Impedance STIFFNESS decreased to " << yellow_key << interim_impedance_stiffness <<  color_key << std::endl;
            }
            else
            {
              ROS_WARN("Impedance STIFFNESS not changed: %f", interim_impedance_stiffness);
            }
            break;
          default:
            ROS_WARN("Impedance STIFFNESS not changed: %f", interim_impedance_stiffness);

            break;
          }
        }
        break;

      case ' ':
        print_command_keys();
        break;
      default:
        ROS_ERROR("Unrecognized Command");;
        break;
      }


      interim_state = (uint64_t) state;

      std::this_thread::sleep_for(std::chrono::milliseconds(100));

    }
    else
    {
      std::cout << yellow_key << state_labels[state] << color_key << " is the state currently active" << std::endl << std::endl;
    } // else

  } // while

}

/*
void smartbox_interface::ROS_subscribe_callback(const esmacat_pkg::esmacat_sensor msg)
{
  //Display data from hard real-time loop to the the terminal.
  // ROS_INFO(" Enc:[%i]",msg.encoder);
}
*/

// Print commands on terminal
void smartbox_interface::print_command_keys()
{
  std::cout << boldred_key << "\nCOMMAND KEYS:"<< color_key << std::endl;
  std::cout << blue_key << "\'x\'" << color_key << ": EXIT" << "\n";
  std::cout << blue_key << "\'s\'" << color_key << ": STOP mode"<< "\n";
  std::cout << blue_key << "\'c\'" << color_key << ": CURRENT mode"<< "\n";
  std::cout << blue_key << "\'t\'" << color_key << ": TORQUE mode"<< "\n";
  std::cout << blue_key << "\'n\'" << color_key << ": NULL-TORQUE mode" << "\n";
  std::cout << blue_key << "\'g\'" << color_key << ": GRAVITY mode"<< "\n";
  std::cout << blue_key << "\'f\'" << color_key << ": FREEZE mode"<< "\n";
  std::cout << blue_key << "\'i\'" << color_key << ": IMPEDANCE mode"<< "\n";
  std::cout << blue_key << "\'d+\'" << color_key << ": increase DAMPING"<< "\n";
  std::cout << blue_key << "\'d-\'" << color_key << ": decrease DAMPING"<< "\n";
  std::cout << blue_key << "\'k+\'" << color_key << ": increase STIFFNESS"<< "\n";
  std::cout << blue_key << "\'k-\'" << color_key << ": decrease STIFFNESS"<< "\n";

  std::cout << blue_key << "\'ENTER\'" << color_key << ": SHOW current settings and command keys\n"<< "\n";
}

void smartbox_interface::ROS_subscribe_callback(const agree_esmacat_pkg::agree_esmacat_status msg)
{
  //Display data from hard real-time loop to the the terminal.
  if( (msg.elapsed_time)%100==0){
    //ROS_INFO("Received: %f",msg.loadcell_torque[1]);
    //ros::shutdown();
  }
}

timespec smartbox_interface::diff(timespec start, timespec end)
{
  timespec temp;
  if ((end.tv_nsec-start.tv_nsec)<0) {
    temp.tv_sec = end.tv_sec-start.tv_sec-1;
    temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
  } else {
    temp.tv_sec = end.tv_sec-start.tv_sec;
    temp.tv_nsec = end.tv_nsec-start.tv_nsec;
  }
  return temp;
}
