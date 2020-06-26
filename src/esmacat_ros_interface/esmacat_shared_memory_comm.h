#ifndef ESMACAT_SHARED_MEMORY_COMM_H
#define ESMACAT_SHARED_MEMORY_COMM_H
#define DEFAULT_ROS_KEY_ID  2108        // street number of Harmonic Bionics

#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdio.h>
#include <iostream>

struct esmacat_sensor{

};

struct esmacat_command{

};

/** Holds all values pertaining to the torque control loop */
struct joint_torque_control_status_t
{
    /** Feedforward torque in milli-Nm that is applied in addition to the PID output */
    float feedforward_demanded_torque_mNm = 0;
    /** Torque setpoint in milli-Nm that is provided as an input to torque control loop */
    float desired_torque_mNm;
    /** Difference between the torque setpoint and the measured torque in milli-Nm */
    float loadcell_error_mNm;
    /** Torque output by the Proportional control in the PID */
    float feedback_p_torque_mNm;
    /** Torque output by the Integral control in the PID */
    float feedback_i_torque_mNm;
    /** Torque output by the Derivative control in the PID */
    float feedback_d_torque_mNm;
    /** Torque in milli-Nm output by the PID */
    float pid_output_torque_mNm;
    /** Feedforward current setpoint for the ESCON : ranges from -1 to +1 */
    float setpoint_feedforward;
    /** Current setpoint for the ESCON computed from the PID: ranges from -1 to +1 */
    float setpoint_feedback;
    /** Final current setpoint that includes both the feedforward and the PID output:
     * ranges from -1 to + 1 */
    float setpoint;
    /** Initialize all values of the struct to 0 */
    joint_torque_control_status_t()
        {
        feedforward_demanded_torque_mNm =0;
        desired_torque_mNm=0;
        loadcell_error_mNm=0;
        feedback_p_torque_mNm=0;
        feedback_i_torque_mNm=0;
        feedback_d_torque_mNm=0;
        pid_output_torque_mNm=0;
        setpoint_feedforward=0;
        setpoint_feedback=0;
        setpoint=0;
    }
};

/** Holds all values pertaining to the impedance control loop */
struct joint_impedance_control_status_t
{
    /** Position setpoint for impedance control in radians */
    float impedance_control_setpoint_rad;
    /** Difference between the position setpoint and the measured
     * setpoint in radians */
    float impedance_control_error_rad;
    /** Output torque of the PD impedance control in milli-Nm */
    float impedance_control_torque_mNm;
    /** Torque applied due to gravity in milli-Nm */
    float gravity_torque_mNm;
    /** Torque required to compensate for friction in milli-Nm */
    float friction_comp_torque_mNm;
    float soft_stop_torque_mNm;
    /** Torque setpoint in milli-Nm that is provided as an output of impedance control */
    float filtered_setpoint_torque_mNm;
    /** Proportional gain for the impedance control loop */
    float impedance_control_k_mNm_per_rad;
    /** Damping gain for the impedance control loop */
    float impedance_control_d_mNm_s_per_rad;

    joint_impedance_control_status_t()
        {
        impedance_control_setpoint_rad = 0;
        impedance_control_error_rad =0;
        impedance_control_torque_mNm=0;
        gravity_torque_mNm=0;
        friction_comp_torque_mNm=0;
        soft_stop_torque_mNm=0;
        filtered_setpoint_torque_mNm=0;
        impedance_control_k_mNm_per_rad = 0;
        impedance_control_d_mNm_s_per_rad = 0;
    }
};

/** Holds all the readings of the actuator for each joint **/
struct joint_status_t
{
    /** Index of the joint within the Harmony */
    int joint_index;
    /** Error in ESCON (1), No errors in ESCON (0); IN_MD_FAULT on ESCON datasheet */
    int fault;
    /** Loadcell reading for the joint in milli-Nm */
    float loadcell_torque_mNm;
    /** Filtered Loadcell reading for the joint in milli-Nm */
    float filtered_load_mNm;
    /** Unfiltered reading from the incremental encoder in counts per turn */
    int32_t unfiltered_incremental_encoder_reading_cpt;
    /** Reading (with offset applied) from the incremental encoder in counts per turn */
    int incremental_encoder_position_cpt;
    /** Reading (with offset applied) from the incremental encoder in degrees*/
    float incremental_encoder_reading_degrees;
    /** Reading (with offset applied) from the incremental encoder in counts radians */
    float incremental_encoder_position_radians;
    /** Velocity derived from incremental encoder */
    float velocity_computed_rad_per_s;
    /** Velocity obtained from EPOS */
    float velocity_rad_per_s;
    /** Default initialization of all the readings (loadcell readings, absolute encoder readings
     * and incremental encoder) for the joint to 0; Joint index is set to -1
     */
    joint_status_t()
    {
        joint_index = -1;
        fault=0;
        loadcell_torque_mNm=0;
        filtered_load_mNm=0;
        unfiltered_incremental_encoder_reading_cpt=0;
        incremental_encoder_position_cpt=0;
        incremental_encoder_reading_degrees=0;
        incremental_encoder_position_radians=0;
        velocity_computed_rad_per_s=0;
        velocity_rad_per_s=0;
    }

};


class esmacat_shared_memory_comm
{
    struct shared_memory_packet {
        float           analog_input[16];
        bool            stop = false;
        double          elapsed_time = 0;
        uint64_t        loop_cnt = 0;
        uint64_t        state = 1;
        esmacat_command command;
        esmacat_sensor  sensor;
        joint_status_t  joint_status;

    };
private:
    key_t key;
    bool is_the_shared_memory_detached= 0;
    int shmid = 0;
public:
//    static int number_of_process_attached_in_shared_memory;
    shared_memory_packet* data;
    esmacat_shared_memory_comm();
    ~esmacat_shared_memory_comm();
    void init();
    void change_shared_memory_key(key_t k){key= k;} // only use this function before init
    void detach_shared_memory();
};

#endif // ESMACAT_SHARED_MEMORY_COMM_H

