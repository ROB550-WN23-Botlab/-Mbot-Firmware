#ifndef MBOT_H
#define MBOT_H

#include <pico/stdlib.h>
#include <pico/mutex.h>
#include <pico/multicore.h>
#include <pico/time.h>
#include <rc/motor/motor.h>
#include <rc/encoder/encoder.h>
#include <rc/motor/motor.h>
#include <rc/defs/common_defs.h>
#include <rc/defs/mbot_diff_defs.h>
#include <rc/fram/fram.h>
#include <rc/math/filter.h>
#include <rc/mpu/mpu.h>
#include <comms/common.h>
#include <comms/protocol.h>
#include <comms/listener.h>
#include <comms/topic_data.h>
#include <comms/mbot_messages.h>

#include <math.h>
#include <inttypes.h>

// Hardware info
#define MAX_FWD_VEL 0.8 // max forward speed (m/s)
#define MESSAGE_CONFIRMATION_CHANNEL "MSG_CONFIRM"
#define MAX_TURN_VEL 2.5 // max turning speed (rad/s)

// TODO: Enter the polarity values for your motors and encoders
#define LEFT_ENC_POL 1
#define RIGHT_ENC_POL -1
#define LEFT_MOTOR_POL -1
#define RIGHT_MOTOR_POL -1

// TODO: Populate with calibration data (recommended to generate these for reverse direction as well)
#define SLOPE_L 1.0
#define SLOPE_R 1.0
#define INTERCEPT_L -0.0
#define INTERCEPT_R -0.0

// TODO: Decide which controller is used, open loop = 1, PID = 0
#define OPEN_LOOP 0

// data to hold current mpu state (not used)
static rc_mpu_data_t mpu_data;
static i2c_inst_t *i2c;

uint64_t timestep_us = 0;

// data to hold calibration coefficients
float coeffs[4];

// data to hold the PID values
static mbot_pid_gains_t mbot_pid_gains;

typedef struct pid_parameters pid_parameters_t;
struct pid_parameters
{
    float kp;
    float ki;
    float kd;
    float dFilterHz;
};

float clamp_duty(float duty);

// data to hold the IMU results
mbot_imu_t current_imu = {0};
// data to hold the received timestamp
timestamp_t received_time = {0};
// current odometry state
odometry_t current_odom = {0};

// previous encoder states
mbot_encoder_t previous_encoders = {0};

// current encoder states
mbot_encoder_t current_encoders = {0};
// current body frame command
mbot_motor_command_t current_cmd = {0};


// Example filter and PID parameter initialization
 
rc_filter_t left_wheel_pid_filter;
rc_filter_t right_wheel_pid_filter;

pid_parameters_t pid_params = {
.kp = 1.2,
.ki = 1.8,
.kd = 0.0,
.dFilterHz = 25.0
 };
 

rc_filter_t left_pid;
rc_filter_t right_pid;
rc_filter_t fwd_vel_pid;
rc_filter_t turn_vel_pid;

pid_parameters_t left_pid_params = {
    .kp = 1.0,
    .ki = 0.0,
    .kd = 0.0,
    .dFilterHz = 25.0,
};
pid_parameters_t right_pid_params = {
    .kp = 1.0,
    .ki = 0.0,
    .kd = 0.0,
    .dFilterHz = 25.0,
};
pid_parameters_t fwd_vel_pid_params = {
    .kp = 1.0,
    .ki = 0.0,
    .kd = 0.0,
    .dFilterHz = 10.0,
};
pid_parameters_t turn_vel_pid_params = {
    .kp = 1.0,
    .ki = 0.0,
    .kd = 0.0,
    .dFilterHz = 10.0,
};

float clamp_duty(float duty);

#endif
