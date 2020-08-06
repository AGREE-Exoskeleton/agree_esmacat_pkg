#include "testbed_ros_interface.h"

/************************/
/* ROS Publisher Thread */
/************************/

void testbed_ros_interface::ROS_publish_thread(){

  // Declare timespec variables
  struct timespec t0,temp,timestamp;
  // Get t0
  clock_gettime( CLOCK_REALTIME, &t0);

  //Declare a message and setup the publisher for that message
  ros::NodeHandle n;
  ros::Rate loop_rate(100);
  agree_esmacat_pkg::agree_esmacat_command msg;

  // Create Publisher Object and Topic
  ros::Publisher pub_esmacat_write = n.advertise<agree_esmacat_pkg::agree_esmacat_command>("esmacat/command",1000);

  //Variables that setup the publishing loop
  int interim_roscount = 0;
  double sine = 0;

  openfile();

  while (ros::ok()){

    msg.command     = interim_status;
    msg.damping_d = interim_impedance_damping;
    msg.stiffness_k = interim_impedance_stiffness;
    sine = sin(2*3.1415*(double)interim_roscount++/500.0); //  sine wave = sin(2*pi*f*t/1000) = sin(2*3.1415*2Hz*timestamp/1kHz)
    msg.setpoint = interim_setpoint; //M_PI/4*sine;                             // setpoint = amplitude * sine wave

    // Compute timestamp
    clock_gettime( CLOCK_REALTIME, &temp);
    timestamp = diff(t0,temp);
    msg.timestamp = (float)(timestamp.tv_sec);

    pub_esmacat_write.publish(msg);

    write2file();

    loop_rate.sleep();
  }

}

/************************/
/* ROS Subscriber Thread */
/************************/


void testbed_ros_interface::ROS_subscribe_thread(){

  //Setup a subscriber that will get data from other ROS nodes
  ros::MultiThreadedSpinner spinner(1); // Use 4 threads

  ros::NodeHandle n;

  ros::Subscriber subscriber = n.subscribe("esmacat/status", 1000, &testbed_ros_interface::ROS_subscribe_callback, this);

  spinner.spin();
}

/************************/
/* ROS Command Thread */
/************************/

void testbed_ros_interface::ROS_command_thread(){

  //Initialize Robot status
  char c;
  string inputString;
  uint8_t state(STOP);


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
          interim_swap_state = true;
        }
        else
        {
          std::cout << yellow_key << "Already in STOP mode" << color_key <<  std::endl;
          interim_swap_state = false;
        }
        break;
      case 'n': case 'N':
        if (state != NULLTORQUE)
        {
          std::cout << green_key << "Quick-swapped to NULL-TORQUE mode!" << color_key << std::endl;
          state = NULLTORQUE;
          interim_swap_state = true;
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
          interim_swap_state = true;
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
          interim_swap_state = true;
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
          interim_swap_state = true;
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
          interim_swap_state = true;
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
          interim_swap_state = true;
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
          interim_swap_state = true;
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
          interim_swap_state = true;
        }
        else
        {
          std::cout << yellow_key << "Already in HOMING mode" << color_key <<  std::endl;
        }
        break;

      case 'x': case 'X':

        if (state == STOP or state == EXIT)
        {   interim_swap_state = true;
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

            interim_impedance_damping += 0.1;
            std::cout << green_key << "Impedance DAMPING increased to " << yellow_key << interim_impedance_damping <<  color_key << std::endl;

            break;
          case '-':
            if(interim_impedance_damping > 0)
            {
              interim_impedance_damping -= 0.1;
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


      interim_status = (uint64_t) state;

      if(!interim_status) {
          closefile();
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          ros::shutdown();
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(100));



    }
    else
    {
      std::cout << yellow_key << state_labels[state] << color_key << " is the state currently active" << std::endl << std::endl;
    } // else

  } // while

}

/***************************/
/* Adaptive Control Thread */
/***************************/

void testbed_ros_interface::adaptive_control_thread(){

    //TODO: Change initial interim_setpoint
    //interim_setpoint = interim_position;

    while (ros::ok()){

        // State Impedance - used for testbed tests
        if(interim_status == IMPEDANCE)
        {

            float interim_time_exercise;

            switch(interim_exercise_status){

            // First case
            case REST:

                // Update Impedance Paramters
                interim_impedance_stiffness = 30.0;
                interim_impedance_damping   = 3.0;

                // Increase counter
                interim_exercise_counter++;

                // Timeout Condition
                if(interim_exercise_counter > TRIGGER_REST_TIMEOUT)
                {
                    // If the joint is closer to START wrt STOP -> Set exercise setpoint
                    if (abs(interim_position - EXERCISE_START) < abs(interim_position - EXERCISE_STOP))
                    {
                        // Setpoint set to STOP
                        interim_setpoint_final = EXERCISE_STOP;
                    }
                    else
                    {
                        // Setpoint is START
                        interim_setpoint_final = EXERCISE_START;
                    }

                    // Exit State
                    interim_exercise_status = WAIT;
                    interim_exercise_counter = 0;

                    interim_setpoint = interim_position;
                    interim_setpoint_start = interim_position;

                    // Update Impedance Parameters
                    interim_impedance_stiffness = 0.0;
                    interim_impedance_damping = 0.1;
                }
                break;

            case WAIT:

                // Follow Trajectory
                interim_setpoint = interim_position;

                // Increase counter
                interim_exercise_counter++;

                // Trigger Condition + Timeout Condition
                if(abs(interim_position - interim_setpoint_start) >= TRIGGER_THRESHOLD || interim_exercise_counter > TRIGGER_TIMEOUT)
                {
                    // Update Impedance Parameters
                    interim_impedance_stiffness = 5.0;
                    interim_impedance_damping = 0.5;

                    // Save Offset
                    interim_elapsed_time_offset = interim_elapsed_time;
                    interim_position_offset = interim_position;

                    // Exit State
                    interim_exercise_status = MOVE_UP;
                    interim_exercise_counter = 0;
                }

            break;

            case MOVE_UP:

                // Compute time variable
                interim_time_exercise = (interim_elapsed_time-interim_elapsed_time_offset);

                /**
                // Tanh(t)
                // interim_setpoint = interim_position_offset+(interim_setpoint_final-interim_position_offset)/2*(tanh((interim_elapsed_time-interim_elapsed_time_offset)/100-5)+1);

                // Sine(2*pi*w*t)
                //interim_setpoint = 5 + interim_sign*5*sin((2*M_PI*((double)interim_time_exercise++/1000.0))-M_PI/2); //  sine wave = sin(2*pi*f*t/1000) = sin(2*3.1415*2Hz*timestamp/1kHz)
                **/
                // 5th order
                interim_setpoint = interim_position_offset+static_cast<float>((interim_setpoint_final-interim_position_offset)*
                                    (10*pow(interim_time_exercise/interim_duration,3)
                                     -15*pow(interim_time_exercise/interim_duration,4)
                                     +6*pow(interim_time_exercise/interim_duration,5)));

                // TODO: Update Stiffness & Damping

                // Reach Condition
                if(interim_position > (interim_setpoint_final - TRIGGER_THRESHOLD/4))
                {
                    // Exit State
                    interim_exercise_status = REST;
                    interim_setpoint = interim_position;
                }

                break;

            case MOVE_DOWN:

                // Compute time variable
                interim_time_exercise = (interim_elapsed_time-interim_elapsed_time_offset);

                /**
                // Tanh(t)
                // interim_setpoint = interim_position_offset+(interim_setpoint_final-interim_position_offset)/2*(tanh((interim_elapsed_time-interim_elapsed_time_offset)/100-5)+1);

                // Sine(2*pi*w*t)
                //interim_setpoint = 5 + interim_sign*5*sin((2*M_PI*((double)interim_time_exercise++/1000.0))-M_PI/2); //  sine wave = sin(2*pi*f*t/1000) = sin(2*3.1415*2Hz*timestamp/1kHz)
                **/
                // 5th order
                interim_setpoint = interim_position_offset+static_cast<float>((interim_setpoint_final-interim_position_offset)*
                                    (10*pow(interim_time_exercise/interim_duration,3)
                                     -15*pow(interim_time_exercise/interim_duration,4)
                                     +6*pow(interim_time_exercise/interim_duration,5)));

                // TODO: Update Stiffness & Damping

                // Reach Condition
                if(interim_position < (interim_setpoint_final + TRIGGER_THRESHOLD/4))
                {
                    // Exit State
                    interim_exercise_status = REST;
                    interim_setpoint = interim_position;
                }

                break;
            }

        }

        else{

            // During other modes, update setpoint to interim_position -> Motor follows movement
            interim_setpoint = interim_position;
        }

        // ~100 Hz Control Frequency
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

}


// Print commands on terminal
void testbed_ros_interface::print_command_keys()
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

void testbed_ros_interface::ROS_subscribe_callback(const agree_esmacat_pkg::agree_esmacat_status msg)
{
    interim_position = msg.encoder_position[0];
    interim_speed    = msg.encoder_speed[0];
    interim_torque   = msg.loadcell_torque[0];
    interim_elapsed_time = msg.elapsed_time;
  //Display data from hard real-time loop to the the terminal.
  if( (msg.elapsed_time)%100==0){
    //ROS_INFO("Received: %f",msg.loadcell_torque[1]);
    //ros::shutdown();
  }
}

timespec testbed_ros_interface::diff(timespec start, timespec end)
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

// //////////////////////////////////////////////////////////////////
// ////////////// Log file Initialization  //////////////////////////
/// \brief testbed_ros_interface::openfile
///
void testbed_ros_interface::openfile(){
    time_t t = time(nullptr);   // get time now
    struct tm * now = localtime( & t );
    char buffer [80];
    strftime (buffer,80,"AGREE-ROS-Interface-%Y-%m-%d-%H-%M-%S.csv",now);
    log.open (buffer);
    if(log.is_open())
    {
        ROS_INFO("Log file opened successfully");

        log << endl

                << "J_elapsed_time"     << ","
                << "J_status"           << ","
                << "J_position_rad"     << ","
                << "J_velocity_rad_s"   << ","
                << "J_torque_mNm"       << ","

                << "I_setpoint_rad"     << ","
                << "I_stiffness"          << ","
                << "I_damping"          << ","
                << "I_duration"         << ","
                << "I_amplitude"        << ","

                << "I_score_k"          << ","
                << "I_score_t"          << ","
                ;
    }

}

// //////////////////////////////////////////////////////////////////
// ////////////// Log file Write  ///////////////////////////////////
void testbed_ros_interface::write2file(){

    log << endl

            << interim_elapsed_time << ","
            << interim_status << ","
            << interim_position << ","
            << interim_speed << ","
            << interim_torque << ","

            << interim_setpoint << ","
            << interim_impedance_stiffness << ","
            << interim_impedance_damping << ","
            << interim_duration << ","
            << interim_amplitude << ","

            << interim_score_k << ","
            << interim_score_t << ",";
}

void testbed_ros_interface::closefile(){
    log.close();
    ROS_INFO("Log file closed successfully");

}


