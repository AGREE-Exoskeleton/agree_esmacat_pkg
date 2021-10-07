#ifndef AGREE_ESMACAT_SHARED_MEMORY_COMM_H
#define AGREE_ESMACAT_SHARED_MEMORY_COMM_H
#define DEFAULT_ROS_KEY_ID  202020

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdio.h>
#include <iostream>
#include "esmacat_applications/agree_esmacat_rt/agree_joint_structs.h" // TODO: Fix this dependency in a smarter way

/*****************************************************************************************
 * CLASSES
 ****************************************************************************************/
class agree_shared_memory_comm
{
    struct shared_memory_packet {

        joint_values_t              joint_values[9];
        torque_control_terms_t      torque_control_terms[9];
        impedance_control_terms_t   impedance_control_terms[9];
        impedance_control_command_t impedance_control_command[9];

        arm_weight_compensation_config_t arm_weight_compensation_config;

        double elapsed_time_ms = 0;
        int last_joint_index = 0;
        int err_msg_count = 0;
        bool use_ros = false;

        bool stop = false;
        robot_control_mode_t control_mode_command = robot_control_mode_t::standby;
        robot_control_mode_t control_mode_status = robot_control_mode_t::standby;

        int exercise_num = 0;
        int right_exercise_num = 0 ;
        int left_exercise_num = 0;
        uint64_t loop_cnt = 0;              // count of loop

    };

private:
    key_t key;
    bool is_the_shared_memory_detached= 0;
    int shmid = 0;
public:
//    static int number_of_process_attached_in_shared_memory;
    shared_memory_packet* data;
    agree_shared_memory_comm();
    ~agree_shared_memory_comm();
    bool init();
    void change_shared_memory_key(key_t k){key= k;} // only use this function before init
    key_t get_shared_memory_key(){return key;}
    void detach_shared_memory();

    void set_esmacat_command(robot_control_mode_t command){data->control_mode_command = command;}
    void set_esmacat_status(robot_control_mode_t status){data->control_mode_status = status;}
    void set_use_ros(bool boolean){data->use_ros = boolean;}

};

#endif // AGREE_ESMACAT_SHARED_MEMORY_COMM_H

