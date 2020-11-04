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
#include "esmacat_applications/agree_mini_torque_driver/agree_joint_structs.h" // TODO: Fix this dependency in a smarter way

/*****************************************************************************************
 * CLASSES
 ****************************************************************************************/
class agree_shared_memory_comm
{
    struct shared_memory_packet {

        agree_joint_status_t              J_status[9];
        agree_torque_control_status_t     J_torque_status[9];
        agree_impedance_control_status_t  J_impedance_control_status[9];

        agree_impedance_control_command_t impedance_control_config[9];

        agree_weight_compensation_config_t agree_weight_config;

        double elapsed_time_ms = 0;
        int last_joint_index = 0;
        int err_msg_count = 0;
        bool is_single_joint = 0;

        bool stop = false;
        robot_control_mode_t mode = robot_control_mode_t::standby;
        int exercise_num = 0;
        int right_exercise_num = 0 ;
        int left_exercise_num = 0;
        uint64_t loop_cnt = 0;              // count of loop
        uint16_t agree_command;
        uint64_t agree_status;

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

    void set_esmacat_command(uint64_t command){data->agree_command = command;}
    void set_esmacat_status(uint64_t status){data->agree_status = status;}
    void set_esmacat_mode(robot_control_mode_t mode){data->mode = mode;}
};

#endif // AGREE_ESMACAT_SHARED_MEMORY_COMM_H

