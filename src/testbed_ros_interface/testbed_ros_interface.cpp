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

      msg.command             = interim_command;
      msg.weight_assistance   = interim_weight_assistance;

      msg.setpoint.clear();
      msg.damping_d.clear();
      msg.stiffness_k.clear();

      for(int joint_index=0;joint_index<N_DOFS_MAX;joint_index++){
          msg.setpoint.push_back(interim_setpoint);
          msg.damping_d.push_back(interim_impedance_damping);
          msg.stiffness_k.push_back(interim_impedance_stiffness);
      }


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
  uint16_t commanded_state(STOP);


  print_command_keys();

  while (ros::ok()){


    // Get character
    c = cin.get();

    if(c != '\n'){
      switch(c){

      case 's': case 'S':
        if (commanded_state != STOP)
        {
          std::cout << green_key << "Quick-swapped to STOP mode!" << color_key << std::endl;
          commanded_state = STOP;
          interim_swap_state = true;
        }
        else
        {
          std::cout << yellow_key << "Already in STOP mode" << color_key <<  std::endl;
          interim_swap_state = false;
        }
        break;
      case 'n': case 'N':
        if (commanded_state != NULLTORQUE)
        {
          std::cout << green_key << "Quick-swapped to NULL-TORQUE mode!" << color_key << std::endl;
          commanded_state = NULLTORQUE;
          interim_swap_state = true;
        }
        else
        {
          std::cout << yellow_key << "Already in NULL-TORQUE mode" << color_key <<  std::endl;
        }
        break;

      case 't': case 'T':
        if (commanded_state != TORQUE)
        {
          std::cout << green_key << "Quick-swapped to TORQUE mode!" << color_key << std::endl;
          commanded_state = TORQUE;
          interim_swap_state = true;
        }
        else
        {
          std::cout << yellow_key << "Already in PASSIVE mode" << color_key <<  std::endl;
        }
        break;

      case 'e': case 'E':
        if (commanded_state != IMPEDANCE_EXT)
        {
          std::cout << green_key << "Quick-swapped to IMPEDANCE_EXT mode!" << color_key << std::endl;
          commanded_state = IMPEDANCE_EXT;
          interim_swap_state = true;
        }
        else
        {
          std::cout << yellow_key << "Already in IMPEDANCE_EXT mode" << color_key <<  std::endl;
        }
        break;

      case 'w': case 'W':
        if (commanded_state != WEIGHT)
        {
          std::cout << green_key << "Quick-swapped to ANTI-GRAVITY mode!" << color_key << std::endl;
          commanded_state = WEIGHT;
          interim_swap_state = true;
        }
        else
        {
          std::cout << yellow_key << "Already in ANTI-GRAVITY mode" << color_key <<  std::endl;
        }
        break;

      case 'g': case 'G':
        if (commanded_state != GRAVITY)
        {
          std::cout << green_key << "Quick-swapped to TRANSPARENT mode!" << color_key << std::endl;
          commanded_state = GRAVITY;
          interim_swap_state = true;
        }
        else
        {
          std::cout << yellow_key << "Already in TRANSPARENT mode" << color_key <<  std::endl;
        }
        break;

      case 'r': case 'R':
        if (commanded_state != RESISTIVE)
        {
          std::cout << green_key << "Quick-swapped to RESISTIVE mode!" << color_key << std::endl;
          commanded_state = RESISTIVE;
          interim_swap_state = true;
        }
        else
        {
          std::cout << yellow_key << "Already in RESISTIVE mode" << color_key <<  std::endl;
        }
        break;

      case 'c': case 'C':
        if (commanded_state != CHALLENGING)
        {
          std::cout << green_key << "Quick-swapped to CHALLENGING mode!" << color_key << std::endl;
          commanded_state = CHALLENGING;
          interim_swap_state = true;
        }
        else
        {
          std::cout << yellow_key << "Already in CHALLENGING mode" << color_key <<  std::endl;
        }
        break;


      case 'f': case 'F':
        if (commanded_state != FREEZE)
        {
          std::cout << green_key << "Quick-swapped to FREEZE mode!" << color_key << std::endl;
          commanded_state = FREEZE;
          interim_swap_state = true;
        }
        else
        {
          std::cout << yellow_key << "Already in FREEZE mode" << color_key <<  std::endl;
        }
        break;
      case 'i': case 'I':
        if (commanded_state != IMPEDANCE)
        {
          std::cout << green_key << "Quick-swapped to IMPEDANCE mode!" << color_key << std::endl;
          commanded_state = IMPEDANCE;
          interim_swap_state = true;
        }
        else
        {
          std::cout << yellow_key << "Already in IMPEDANCE mode" << color_key <<  std::endl;
        }
        break;

      case 'p': case 'P':
        if (commanded_state != POSITION)
        {
          std::cout << green_key << "Quick-swapped to POSITION mode!" << color_key << std::endl;
          commanded_state = POSITION;
          interim_swap_state = true;
        }
        else
        {
          std::cout << yellow_key << "Already in POSITION mode" << color_key <<  std::endl;
        }
        break;

      case 'h': case 'H':
        if (commanded_state != HOMING)
        {
          std::cout << green_key << "Quick-swapped to HOMING mode!" << color_key << std::endl;
          commanded_state = HOMING;
          interim_swap_state = true;
        }
        else
        {
          std::cout << yellow_key << "Already in HOMING mode" << color_key <<  std::endl;
        }
        break;



      case 'x': case 'X':

        if (commanded_state == STOP or commanded_state == EXIT)
        {   interim_swap_state = true;
          commanded_state = EXIT;
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

            interim_impedance_damping += 0.25;
            std::cout << green_key << "Impedance DAMPING increased to " << yellow_key << interim_impedance_damping <<  color_key << std::endl;

            break;
          case '-':
            if(interim_impedance_damping > 0)
            {
              interim_impedance_damping -= 0.25;
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

            interim_impedance_stiffness += 2.5;
            std::cout << green_key << "Impedance STIFFNESS increased to " << yellow_key << interim_impedance_stiffness <<  color_key << std::endl;

            break;
          case '-':
            if(interim_impedance_stiffness > 0)
            {
              interim_impedance_stiffness -= 2.5;
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

      case 'a': case 'A':
          if(c != '\n'){
            std::cout << yellow_key << "WEIGHT Assistance change selection: " << color_key << std::endl;
            c = cin.get();
            switch(c){
            case '+':

              interim_weight_assistance += 0.05;
              std::cout << green_key << "WEIGHT Assistance increased to " << yellow_key << interim_weight_assistance <<  color_key << std::endl;

              break;
            case '-':
              if(interim_weight_assistance > -1.0 && interim_weight_assistance < 2.0 )
              {
                interim_weight_assistance -= 0.05;
                std::cout << green_key << "WEIGHT Assistance decreased to " << yellow_key << interim_weight_assistance <<  color_key << std::endl;
              }
              else
              {
                ROS_WARN("WEIGHT Assistance not changed: %f", interim_weight_assistance);
              }
              break;
            default:
              ROS_WARN("WEIGHT Assistance not changed: %f", interim_weight_assistance);

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

      interim_command = (uint8_t) commanded_state;

      if(!interim_command) {
          closefile();
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          ros::shutdown();
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(100));



    }
    else
    {
      std::cout << yellow_key << robot_mode_labels[commanded_state] << color_key << " is the state currently active" << std::endl << std::endl;
    } // else

  } // while

}

/***************************/
/* Adaptive Control Thread */
/***************************/

void testbed_ros_interface::control_thread(){

    //TODO: Change initial interim_setpoint
    //interim_setpoint = interim_position;

    float P0,P1,P2,P3,P4,P5;
    float interim_time_exercise;
    float interim_setpoint_exercise;

    P0 = EXERCISE_START;
    P2 = 0.0;
    P3 = 3.0; //3rd Order
    P4 = P2+EXERCISE_DURATION;
    P5 = 3.0; //3rd Order
    P1 = EXERCISE_AMPLITUDE/pow(EXERCISE_DURATION/2,(P3+P5));


    while (ros::ok()){

        // Compute time variable
        interim_time_exercise = (interim_elapsed_time-interim_elapsed_time_offset);

        // Beta-Function
        interim_setpoint_exercise = P0+P1*pow((interim_time_exercise-P2),P3)*pow((P4-interim_time_exercise),P5);

        switch(interim_command){

        /***************************/
        /*        FREEZE           */
        /***************************/

        case FREEZE:

            // First iteration:
            if(interim_swap_state){
                // interim_setpoint = last position
                interim_setpoint = interim_position;
                // Stiffness = 60 Nm/rad
                interim_impedance_stiffness = 60.0;
                // Damping = 15 Nm/rad^2 -> derived from stiffness
                interim_impedance_damping = STIFFNESS_TO_DAMPING_RATIO*interim_impedance_stiffness;
                // Exit first iteration
                interim_swap_state = false;
            }
            // Damping is derived from stiffness
            interim_impedance_damping = STIFFNESS_TO_DAMPING_RATIO*interim_impedance_stiffness;

            break;

        /***************************/
        /*    GO HOME POSITION     */
        /***************************/

        case POSITION:

            // First iteration:
            if(interim_swap_state){
                // interim time offset = last time
                interim_elapsed_time_offset = interim_elapsed_time;
                // Stiffness = 10 Nm/rad
                interim_impedance_stiffness = 25.0;
                // Assistance = 50%
                interim_weight_assistance = WEIGHT_ASSISTANCE;
                // Exit first iteration
                interim_swap_state = false;
            }
            // interim setpoint = Start position for exercise
            interim_setpoint = EXERCISE_START;
            // Damping = 2.5 Nm/rad^2 -> derived from stiffness
            interim_impedance_damping = STIFFNESS_TO_DAMPING_RATIO*interim_impedance_stiffness;

            break;


        /********************/
        /*    IMPEDANCE     */
        /********************/

        case IMPEDANCE:

            // First Iteration:
            if(interim_swap_state){
                // Stiffness = 60 Nm/rad
                interim_impedance_stiffness = 5.0;
                // Damping = 15 Nm/rad^2 -> derived from stiffness
                interim_impedance_damping = STIFFNESS_TO_DAMPING_RATIO*interim_impedance_stiffness;
                // Exit first iteration
                interim_swap_state = false;
            }
            break;

        /*******************************/
        /* IMPEDANCE EXTERNAL SETPOINT */
        /*******************************/

        case IMPEDANCE_EXT:

            // First iteration:
            if(interim_swap_state){
                // interim_setpoint = last position
                interim_setpoint = interim_position;
                // Assistance = 50%
                interim_weight_assistance = WEIGHT_ASSISTANCE;
                // Stiffness = 60 Nm/rad
                interim_impedance_stiffness = 5.0;
                // Damping = 15 Nm/rad^2 -> derived from stiffness
                interim_impedance_damping = 0.5; //STIFFNESS_TO_DAMPING_RATIO*interim_impedance_stiffness;
                // Time offset = last time
                interim_elapsed_time_offset = interim_elapsed_time;
                // Exit first iteration
                interim_swap_state = false;
                interim_repetition=0;

            }

                // interim_impedance = changed by terminal
                // interim_stiffness = changed by terminal

                // Restart Beta-Function
                if(interim_time_exercise > EXERCISE_DURATION) {
                    interim_elapsed_time_offset = interim_elapsed_time;
                    interim_repetition++;
                    ROS_INFO("Repetition #%d",interim_repetition);
                }
                // Beta-Function
                interim_setpoint = interim_setpoint_exercise;


            break;

        /*******************************/
        /* IMPEDANCE EXTERNAL SETPOINT */
        /*******************************/

        case PASSIVE:

            // First iteration:
            if(interim_swap_state){
                // interim_setpoint = last position
                interim_setpoint = interim_position;
                // Assistance = 50%
                interim_weight_assistance = WEIGHT_ASSISTANCE;
                // Stiffness = 60 Nm/rad
                interim_impedance_stiffness = 50;
                // Damping = 15 Nm/rad^2 -> derived from stiffness
                interim_impedance_damping = 10.0; //STIFFNESS_TO_DAMPING_RATIO*interim_impedance_stiffness;
                // Time offset = last time
                interim_elapsed_time_offset = interim_elapsed_time;
                // Exit first iteration
                interim_swap_state = false;
                interim_repetition=0;

            }

                // interim_impedance = changed by terminal
                // interim_stiffness = changed by terminal

            // Restart Beta-Function
            if(interim_time_exercise > EXERCISE_DURATION) {
                interim_elapsed_time_offset = interim_elapsed_time;
                interim_repetition++;
                ROS_INFO("Repetition #%d",interim_repetition);
            }
                // Beta-Function
                interim_setpoint = interim_setpoint_exercise;


            break;


        /***************************/
        /*         WEIGHT          */
        /***************************/

        case WEIGHT:

            // First iteration:
            if(interim_swap_state){

                // interim time offset = last time
                interim_elapsed_time_offset = interim_elapsed_time;
                // Stiffness = 0 Nm/rad
                interim_impedance_stiffness = 0.0;
                // Damping = 0.1 Nm/rad^2
                interim_impedance_damping = 0.1;
                // Assistance = 50%
                interim_weight_assistance = WEIGHT_ASSISTANCE;
                // Exit first iteration
                interim_swap_state = false;
                interim_repetition=0;

            }

            // Restart Beta-Function
            if(interim_time_exercise > EXERCISE_DURATION) {
                interim_elapsed_time_offset = interim_elapsed_time;
                interim_repetition++;
                ROS_INFO("Repetition #%d",interim_repetition);
            }
            // Beta-Function
            interim_setpoint = interim_setpoint_exercise;

            break;

        /***************************/
        /*        GRAVITY          */
        /***************************/

        case GRAVITY:
            // First iteration:
            if(interim_swap_state){

                // interim time offset = last time
                interim_elapsed_time_offset = interim_elapsed_time;
                // Stiffness = 0 Nm/rad
                interim_impedance_stiffness = 0.0;
                // Damping = 0.1 Nm/rad^2
                interim_impedance_damping = 0.1;
                // Exit first iteration
                interim_swap_state = false;
                interim_repetition=0;

            }

            // Restart Beta-Function
            if(interim_time_exercise > EXERCISE_DURATION) {
                interim_elapsed_time_offset = interim_elapsed_time;
                interim_repetition++;
                ROS_INFO("Repetition #%d",interim_repetition);
            }
            // Beta-Function
            interim_setpoint = interim_setpoint_exercise;

        break;

        case RESISTIVE:
            // First iteration:
            if(interim_swap_state){

                // interim time offset = last time
                interim_elapsed_time_offset = interim_elapsed_time;
                // Stiffness = 0 Nm/rad
                interim_impedance_stiffness = 0.0;
                // Damping = 0.1 Nm/rad^2
                interim_impedance_damping = 2.5;
                // Exit first iteration
                interim_swap_state = false;
                interim_repetition=0;

            }

            // Restart Beta-Function
            if(interim_time_exercise > EXERCISE_DURATION) {
                interim_elapsed_time_offset = interim_elapsed_time;
                interim_repetition++;
                ROS_INFO("Repetition #%d",interim_repetition);
            }
            // Beta-Function
            interim_setpoint = interim_setpoint_exercise;

        break;

        case CHALLENGING:
            // First iteration:
            if(interim_swap_state){

                // interim time offset = last time
                interim_elapsed_time_offset = interim_elapsed_time;
                // Stiffness = 0 Nm/rad
                interim_impedance_stiffness = 0.0;
                // Damping = 0.1 Nm/rad^2
                interim_impedance_damping = 0.5;

                interim_weight_assistance = -1.0;
                // Exit first iteration
                interim_swap_state = false;

                interim_repetition=0;
            }

            // Restart Beta-Function
            if(interim_time_exercise > EXERCISE_DURATION) {
                interim_elapsed_time_offset = interim_elapsed_time;
                interim_repetition++;
                ROS_INFO("Repetition #%d",interim_repetition);
            }
            // Beta-Function
            interim_setpoint = interim_setpoint_exercise;

        break;


        default:
            break;
        }




        // State Impedance - used for testbed tests
        if(interim_command == TRIGGER)
        {

            float impedance_error, impedance_torque;
            bool trigger_condition = false;

            switch(interim_exercise_status){

            // First case
            case REST:

                // Update Impedance Paramters
                interim_impedance_stiffness = 50.0;
                interim_impedance_damping   = 5.0;

                // Increase counter
                interim_exercise_counter++;
                //std::cout << "REST: " << interim_exercise_counter <<endl;

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

                    std::cout << "SETPOINT: " << interim_setpoint_final << endl;

                    // Exit State
                    interim_exercise_status = WAIT;
                    ROS_INFO("Exercise Status: WAIT");
                    interim_exercise_counter = 0;

                    //interim_setpoint = interim_position;
                    interim_setpoint_start = interim_position;

                    // Update Impedance Parameters
                    if(trigger_mode == TRIGGER_POSITION){
                    interim_impedance_stiffness = 0.0;
                    interim_impedance_damping = 0.5;
                    }
                }

                break;

            case WAIT:

                // Follow Trajectory

                // Increase counter
                interim_exercise_counter++;
                //std::cout << "WAIT: " << interim_exercise_counter <<endl;


                // Define Trigger Mode
                if(trigger_mode==TRIGGER_POSITION){
                    interim_setpoint = interim_position;
                    if( abs(interim_position - interim_setpoint_start) >= TRIGGER_THRESHOLD) trigger_condition = true;
                }
                else if(trigger_mode==TRIGGER_TORQUE){
                   if( abs(interim_torque) >= TRIGGER_TORQUE_THRESHOLD) trigger_condition = true;
                }
                // Trigger Condition + Timeout Condition
                if( trigger_condition == true || interim_exercise_counter > TRIGGER_TIMEOUT)
                {
                    impedance_error = (interim_setpoint - interim_position);
                    impedance_torque = impedance_error*interim_impedance_stiffness;

                    // Update Impedance Paramters
                    interim_impedance_stiffness = saved_impedance_stiffness;
                    interim_impedance_damping   = saved_impedance_damping;

                    // Continuity Condition REST-MOVE
                    interim_setpoint = impedance_torque/interim_impedance_stiffness+interim_position;

                    // Save Offset
                    interim_elapsed_time_offset = interim_elapsed_time;
                    interim_position_offset = interim_setpoint;

                    // Exit State
                    if(abs(interim_setpoint_final - EXERCISE_STOP) < 1E-3)
                    {
                        interim_exercise_status = MOVE_UP;
                        ROS_INFO("Exercise Status: MOVE_UP");
                    }
                    else if(abs(interim_setpoint_final - EXERCISE_START) < 1E-3)
                    {
                        interim_exercise_status = MOVE_DOWN;
                        ROS_INFO("Exercise Status: MOVE_DOWN");
                    }
                    interim_exercise_counter = 0;
                }

                break;

            case MOVE_UP:

                // Compute time variable
                interim_time_exercise = (interim_elapsed_time-interim_elapsed_time_offset);

                /**
                // Tanh(t)
                // interim_setpoint = interim_position_offset+(interim_setpoint_final-interim_position_offset)/2*(tanh((interim_elapsed_time-interim_elapsed_time_offset)/100-5)+1);
                **/

                // Sine(2*pi*w*t)
//                interim_setpoint = interim_setpoint_final/2 + interim_setpoint_final/2*sin((2*M_PI*((double)interim_time_exercise++/1000.0))-M_PI/2); //  sine wave = sin(2*pi*f*t/1000) = sin(2*3.1415*2Hz*timestamp/1kHz)

                // 5th order
                interim_setpoint = interim_position_offset+static_cast<float>((interim_setpoint_final-interim_position_offset)*
                                    (10*pow(interim_time_exercise/interim_duration,3)
                                     -15*pow(interim_time_exercise/interim_duration,4)
                                     +6*pow(interim_time_exercise/interim_duration,5)));

                // TODO: Update Stiffness & Damping
                saved_impedance_stiffness = interim_impedance_stiffness;
                saved_impedance_damping   = interim_impedance_damping;


                // Reach Condition
                if(interim_setpoint > (interim_setpoint_final - 1E-4))
                {
                    // Exit State
                    interim_exercise_status = REST;
                    ROS_INFO("Exercise Status: REST");

                    impedance_error = (interim_setpoint - interim_position);
                    impedance_torque = impedance_error*interim_impedance_stiffness;

                    // Update Impedance Paramters
                    interim_impedance_stiffness = 50.0;
                    interim_impedance_damping   = 5.0;

                    // Continuity Condition REST-MOVE
                    interim_setpoint = impedance_torque/interim_impedance_stiffness+interim_position;
                }


                break;

            case MOVE_DOWN:

                // Compute time variable
                interim_time_exercise = (interim_elapsed_time-interim_elapsed_time_offset);

                /**
                // Tanh(t)
                // interim_setpoint = interim_position_offset+(interim_setpoint_final-interim_position_offset)/2*(tanh((interim_elapsed_time-interim_elapsed_time_offset)/100-5)+1);
                **/
                // Sine(2*pi*w*t)
//                interim_setpoint = interim_setpoint_final/2 + interim_setpoint_final/2*sin((2*M_PI*((double)interim_time_exercise++/1000.0))-M_PI/2); //  sine wave = sin(2*pi*f*t/1000) = sin(2*3.1415*2Hz*timestamp/1kHz)

                // 5th order
                interim_setpoint = interim_position_offset+static_cast<float>((interim_setpoint_final-interim_position_offset)*
                                    (10*pow(interim_time_exercise/interim_duration,3)
                                     -15*pow(interim_time_exercise/interim_duration,4)
                                     +6*pow(interim_time_exercise/interim_duration,5)));

                // TODO: Update Stiffness & Damping
                saved_impedance_stiffness = interim_impedance_stiffness;
                saved_impedance_damping   = interim_impedance_damping;

                // Reach Condition
                if(interim_setpoint < (interim_setpoint_final + 1E-4))
                {
                    // Exit State
                    interim_exercise_status = REST;
                    ROS_INFO("Exercise Status: REST");

                    impedance_error = (interim_setpoint - interim_position);
                    impedance_torque = impedance_error*interim_impedance_stiffness;

                    // Update Impedance Paramters
                    interim_impedance_stiffness = 50.0;
                    interim_impedance_damping   = 5.0;

                    // Continuity Condition REST-MOVE
                    interim_setpoint = impedance_torque/interim_impedance_stiffness+interim_position;
                }

                break;
            }

        }

        /*
        else if(interim_command == ADAPTIVE){

            // Compute time variable
            interim_time_exercise = (interim_elapsed_time-interim_elapsed_time_offset);

            // Filtered Derivative Error (Exponential Low-Pass Filter)
            // Filtered Velocity (Exponential Low-Pass Filter)
            // y(k) = (1-a) * y(k-1) + a * x(k)
            double lambda = 0.0005;
            adaptive_filtered_error_rad = lambda*abs(interim_setpoint-interim_position) + (1-lambda)*(adaptive_filtered_error_rad);

            adaptive_impedance_stiffness = adaptive_gain*adaptive_filtered_error_rad + adaptive_forgetting_factor*adaptive_impedance_stiffness;

            interim_setpoint = interim_position_offset + M_PI/4*sin(2*M_PI*(double)interim_time_exercise++/10000.0);
            interim_impedance_stiffness = adaptive_impedance_stiffness;
            interim_impedance_damping = interim_impedance_stiffness/5;
        }





        else if(interim_command == FREEZE){
            if(interim_swap_state){
                // interim_setpoint = last position
                interim_setpoint = interim_position;
                // Stiffness = 60 Nm/rad
                interim_impedance_stiffness = 60.0;
                // Damping = 15 Nm/rad^2 -> derived from stiffness
                interim_impedance_damping = STIFFNESS_TO_DAMPING_RATIO*interim_impedance_stiffness;
                // Exit first iteration
                interim_swap_state = false;
            }
            // Damping is derived from stiffness
            interim_impedance_damping = STIFFNESS_TO_DAMPING_RATIO*interim_impedance_stiffness;
        }
        */





        /*
        else {

            // During other modes, update setpoint to interim_position -> Motor follows movement
            //saved_impedance_stiffness = interim_impedance_stiffness;
            //saved_impedance_damping   = interim_impedance_damping;
            interim_exercise_status   = REST;

            // Save Offset
            interim_elapsed_time_offset = interim_elapsed_time;
            interim_position_offset = interim_setpoint;
            interim_impedance_stiffness = 5.0;
            interim_impedance_damping = 0.5;

            // Clear adaptive errors
            adaptive_filtered_error_rad         = 0.0;
            adaptive_impedance_stiffness        = 5.0;
            adaptive_impedance_damping          = 0.5;
        }
        */

        // ~100 Hz Control Frequency
        // TODO: CLOCKTIME
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

}


// Print commands on terminal
void testbed_ros_interface::print_command_keys()
{
  std::cout << boldred_key << "\nCOMMAND KEYS:"<< color_key << std::endl;
  std::cout << blue_key << "\'x\'" << color_key << ": EXIT" << "\n";
  std::cout << blue_key << "\'s\'" << color_key << ": STOP mode"<< "\n";

  std::cout << blue_key << "\'n\'" << color_key << ": NULL-TORQUE mode" << "\n";
  std::cout << blue_key << "\'f\'" << color_key << ": FREEZE mode"<< "\n";
  std::cout << blue_key << "\'h\'" << color_key << ": HOMING mode"<< "\n";
  std::cout << blue_key << "\'p\'" << color_key << ": POSITION mode"<< "\n";

  std::cout << blue_key << "\'t\'" << color_key << ": TORQUE mode"<< "\n";
  std::cout << blue_key << "\'i\'" << color_key << ": IMPEDANCE EXTERNAL mode"<< "\n";
  std::cout << blue_key << "\'w\'" << color_key << ": ANTI-G mode"<< "\n";
  std::cout << blue_key << "\'g\'" << color_key << ": TRANSPARENT mode"<< "\n";
  std::cout << blue_key << "\'r\'" << color_key << ": RESISTIVE mode"<< "\n";
  std::cout << blue_key << "\'c\'" << color_key << ": CHALLENGING mode"<< "\n";

  cout << endl;
  std::cout << blue_key << "\'d+\'" << color_key << ": increase DAMPING"<< "\n";
  std::cout << blue_key << "\'d-\'" << color_key << ": decrease DAMPING"<< "\n";
  std::cout << blue_key << "\'k+\'" << color_key << ": increase STIFFNESS"<< "\n";
  std::cout << blue_key << "\'k-\'" << color_key << ": decrease STIFFNESS"<< "\n";
  std::cout << blue_key << "\'a+\'" << color_key << ": increase WEIGHT assistance"<< "\n";
  std::cout << blue_key << "\'a-\'" << color_key << ": decrease WEIGHT assistance"<< "\n";
  cout << endl;
  std::cout << blue_key << "\'ENTER\'" << color_key << ": SHOW current settings and command keys\n"<< "\n";
}

void testbed_ros_interface::ROS_subscribe_callback(const agree_esmacat_pkg::agree_esmacat_status msg)
{
    interim_position = msg.joint_position_rad[0];
    interim_speed    = msg.joint_speed_rad_s[0];
    interim_torque   = msg.joint_torque_mNm[0];

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
            << interim_command << ","
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


