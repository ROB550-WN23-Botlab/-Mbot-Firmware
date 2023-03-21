/**
 * This file is the main executable for the MBot firmware.
 */

#include <pico/stdlib.h>
#include <pico/mutex.h>
#include <pico/multicore.h>
#include <pico/time.h>
#include <rc/motor/motor.h>
#include <rc/encoder/encoder.h>
#include <rc/motor/motor.h>
#include <rc/mpu/mpu.h>
#include <comms/common.h>
#include <comms/protocol.h>
#include <comms/listener.h>
#include <comms/topic_data.h>
#include <comms/mbot_messages.h>

#include <math.h>
#include <inttypes.h>
#include "mbot.h"

#define LED_PIN 25
#define MAIN_LOOP_HZ 50.0 // 50 hz loop
#define MAIN_LOOP_PERIOD (1.0f / MAIN_LOOP_HZ)

// data to hold current mpu state
static rc_mpu_data_t mpu_data;

uint64_t timestamp_offset = 0;
uint64_t current_pico_time = 0;

float enc2meters = ((2.0 * PI * WHEEL_RADIUS) / (GEAR_RATIO * ENCODER_RES));

void timestamp_cb(timestamp_t *received_timestamp)
{
    // if we havent set the offset yet
    if (timestamp_offset == 0)
    {
        uint64_t cur_pico_time = to_us_since_boot(get_absolute_time());
        timestamp_offset = received_timestamp->utime - cur_pico_time;
    }
}

void reset_encoders_cb(mbot_encoder_t *received_encoder_vals)
{
    rc_encoder_write(LEFT_MOTOR_CHANNEL, received_encoder_vals->leftticks);
    rc_encoder_write(RIGHT_MOTOR_CHANNEL, received_encoder_vals->rightticks);
}

void reset_odometry_cb(odometry_t *received_odom)
{
    current_odom.utime = received_odom->utime;
    current_odom.x = received_odom->x;
    current_odom.y = received_odom->y;
    current_odom.theta = received_odom->theta;
}

int write_pid_coefficients(i2c_inst_t *i2c)
{
    uint8_t pid_bytes[PID_VALUES_LEN];
    memcpy(pid_bytes, &mbot_pid_gains, PID_VALUES_LEN);
    return rc_write_fram(i2c, PID_VALUES_ADDR, PID_VALUES_LEN, &pid_bytes[0]);
}

void pid_values_cb(mbot_pid_gains_t *received_pid_gains)
{
    memcpy(&mbot_pid_gains, received_pid_gains, sizeof(mbot_pid_gains_t));
    write_pid_coefficients(i2c);
}

void register_topics()
{
    // timesync topic
    comms_register_topic(MBOT_TIMESYNC, sizeof(timestamp_t), (Deserialize)&timestamp_t_deserialize, (Serialize)&timestamp_t_serialize, (MsgCb)&timestamp_cb);
    // odometry topic
    comms_register_topic(ODOMETRY, sizeof(odometry_t), (Deserialize)&odometry_t_deserialize, (Serialize)&odometry_t_serialize, NULL);
    // reset odometry topic
    comms_register_topic(RESET_ODOMETRY, sizeof(odometry_t), (Deserialize)&odometry_t_deserialize, (Serialize)&odometry_t_serialize, (MsgCb)&reset_odometry_cb);
    // IMU topic
    comms_register_topic(MBOT_IMU, sizeof(mbot_imu_t), (Deserialize)&mbot_imu_t_deserialize, (Serialize)&mbot_imu_t_serialize, NULL);
    // encoders topic
    comms_register_topic(MBOT_ENCODERS, sizeof(mbot_encoder_t), (Deserialize)&mbot_encoder_t_deserialize, (Serialize)&mbot_encoder_t_serialize, NULL);
    // reset encoders topic
    comms_register_topic(RESET_ENCODERS, sizeof(mbot_encoder_t), (Deserialize)&mbot_encoder_t_deserialize, (Serialize)&mbot_encoder_t_serialize, (MsgCb)&reset_encoders_cb);
    // motor commands topic
    comms_register_topic(MBOT_MOTOR_COMMAND, sizeof(mbot_motor_command_t), (Deserialize)&mbot_motor_command_t_deserialize, (Serialize)&mbot_motor_command_t_serialize, NULL);
    // PID values topic
    comms_register_topic(MBOT_PIDS, sizeof(mbot_pid_gains_t), (Deserialize)&mbot_pid_gains_t_deserialize, (Serialize)&mbot_pid_gains_t_serialize, (MsgCb)&pid_values_cb);
}

void read_pid_coefficients(i2c_inst_t *i2c)
{
    uint8_t pid_bytes[PID_VALUES_LEN];

    if (rc_read_fram(i2c, PID_VALUES_ADDR, PID_VALUES_LEN, pid_bytes) > 0)
    {
        printf("reading fram success.\n");
        memcpy(&mbot_pid_gains, pid_bytes, PID_VALUES_LEN);
        printf("read gains from fram!\r\n");
    }
    else
    {
        printf("reading PID gains from fram failure.\n");
    }
}

double linearSpeedToDuty_yuantao(double speed,char motor)
{
    if(speed == 0)
        {return 0;}

    // left motor forward
    else if(speed > 0 && motor == 'l')
    {
        double duty = 0.8935848 * speed + 0.10415794;
        return duty<=1 ? duty: 1;
    }

    // left motor backward
    else if(speed < 0 && motor == 'l')
    {
        double duty = 0.9203841 * speed - 0.10476584;
        return duty>=-1 ? duty: -1;
    }

    // right motor forward
    else if(speed > 0 && motor == 'r')
    {
        double duty = 0.92308803 * speed + 0.11279647;
        return duty<=1 ? duty : 1;
    }

    else if(speed < 0 && motor == 'r')
    {
        double duty = 0.92817986 * speed - 0.10170667;
        return duty>=-1 ? duty: -1;
    }

    else
    {
        printf("error in speed to duty");
    }
}


double linearSpeedToDuty(double speed,char motor)
{
    if(speed == 0)
        {return 0;}

    // left motor forward
    else if(speed > 0 && motor == 'l')
    {
        double duty = 1.38420797  * speed + 0.10521247;
        return duty<=1 ? duty: 1;
    }

    // left motor backward
    else if(speed < 0 && motor == 'l')
    {
        double duty = 1.41440438  * speed - 0.09809853;
        return duty>=-1 ? duty: -1;
    }

    // right motor forward
    else if(speed > 0 && motor == 'r')
    {
        double duty = 1.41891136  * speed + 0.13051303;
        return duty<=1 ? duty : 1;
    }

    else if(speed < 0 && motor == 'r')
    {
        double duty = 1.38766877  * speed -0.13458451;
        return duty>=-1 ? duty: -1;
    }

    else
    {
        printf("error in speed to duty");
    }
}


bool timer_cb(repeating_timer_t *rt)
{
    // Read the PID values
    if (comms_get_topic_data(MBOT_PIDS, &mbot_pid_gains))
    {
        uint64_t msg_time = current_pico_time;
        // Print the PID values
        printf("Left: %f, %f, %f, %f", mbot_pid_gains.motor_a_kp,
               mbot_pid_gains.motor_a_ki,
               mbot_pid_gains.motor_a_kd,
               1.0 / mbot_pid_gains.motor_a_Tf);

        // update the PID gains of the left motor PID controller
        rc_filter_pid(&left_pid,
                      mbot_pid_gains.motor_a_kp,
                      mbot_pid_gains.motor_a_ki,
                      mbot_pid_gains.motor_a_kd,
                      1.0 / mbot_pid_gains.motor_a_Tf,
                      1.0 / MAIN_LOOP_HZ);

        // update the PID gains of the right motor PID controller
        rc_filter_pid(&right_pid,
                      mbot_pid_gains.motor_c_kp,
                      mbot_pid_gains.motor_c_ki,
                      mbot_pid_gains.motor_c_kd,
                      1.0 / mbot_pid_gains.motor_c_Tf,
                      1.0 / MAIN_LOOP_HZ);

        // update the PID gains of the body frame controller
        rc_filter_pid(&fwd_vel_pid,
                      mbot_pid_gains.bf_trans_kp,
                      mbot_pid_gains.bf_trans_ki,
                      mbot_pid_gains.bf_trans_kd,
                      1.0 / mbot_pid_gains.bf_trans_Tf,
                      1.0 / MAIN_LOOP_HZ);

        // update the PID gains of the body frame rotation controller
        rc_filter_pid(&turn_vel_pid,
                      mbot_pid_gains.bf_rot_kp,
                      mbot_pid_gains.bf_rot_ki,
                      mbot_pid_gains.bf_rot_kd,
                      1.0 / mbot_pid_gains.bf_rot_Tf,
                      1.0 / MAIN_LOOP_HZ);
    }
    // only run if we've received a timesync message...
    if (comms_get_topic_data(MBOT_TIMESYNC, &received_time))
    {
        uint64_t cur_pico_time = to_us_since_boot(get_absolute_time()) + timestamp_offset;
        uint64_t latency_time = cur_pico_time - current_pico_time;
        current_pico_time = cur_pico_time;
        // first, get the IMU data and send across the wire
        current_imu.utime = cur_pico_time; // received_time.utime;

        // read the encoders
        int enc_cnt_l = LEFT_ENC_POL * rc_encoder_read_count(LEFT_MOTOR_CHANNEL);
        int enc_delta_l = LEFT_ENC_POL * rc_encoder_read_delta(LEFT_MOTOR_CHANNEL);
        int enc_cnt_r = RIGHT_ENC_POL * rc_encoder_read_count(RIGHT_MOTOR_CHANNEL);
        int enc_delta_r = RIGHT_ENC_POL * rc_encoder_read_delta(RIGHT_MOTOR_CHANNEL);


        // copy current encoders data to previous data (these will be used in pid speed control)
        previous_encoders.utime = current_encoders.utime; // received_time.utime;
        previous_encoders.right_delta = current_encoders.right_delta;
        previous_encoders.rightticks = current_encoders.rightticks;
        previous_encoders.left_delta = current_encoders.left_delta;
        previous_encoders.leftticks = current_encoders.leftticks;
        
        
        current_encoders.utime = cur_pico_time; // received_time.utime;
        current_encoders.right_delta = enc_delta_r;
        current_encoders.rightticks = enc_cnt_r;
        current_encoders.left_delta = enc_delta_l;
        current_encoders.leftticks = enc_cnt_l;

        // compute new odometry
        /*************************************************************
         * TODO:
         *      - Populate the odometry messages.
         *          -The struct for odometry_t is defined in comms/include/comms/messages_mb.h (DO NOT EDIT THIS FILE)
         *      - Note that the way we compute the displacement of the motor is from encoder readings and
         *        that we convert encoder readings to meters using the enc2meters variable defined above.
         *      - Use the equations provided in the document to compute the odometry components
         *      - Remember to clamp the orientation between [0, 2pi]!
         *************************************************************/
        float delta_theta; // displacement in meters and rotation in radians
        double delta_x = 0;
        double delta_y = 0;

        /*************************************************************
         * End of TODO
         *************************************************************/
        double rightWheelDistance = current_encoders.right_delta * enc2meters;
        double leftWheelDistance = current_encoders.left_delta * enc2meters;
        
        double previousTheta = current_odom.theta;
        // double previousX = current_odom.x;
        // double previousY = current_odom.y; 
        
        if(rightWheelDistance == leftWheelDistance)
        {
            delta_theta = 0;
            delta_x = (rightWheelDistance) * cos(previousTheta);
            delta_y = (rightWheelDistance) * sin(previousTheta);
        }

        else
        {
            double radiasOfTurn = (WHEEL_BASE/2)*((rightWheelDistance+leftWheelDistance)/
                                                    (rightWheelDistance-leftWheelDistance));
            delta_theta = (rightWheelDistance-leftWheelDistance)/(WHEEL_BASE);
            delta_y = radiasOfTurn*(-cos(previousTheta + delta_theta) + cos(previousTheta));
            delta_x = radiasOfTurn*(sin(previousTheta + delta_theta) - sin(previousTheta));
        }

        current_odom.x=current_odom.x + delta_x;
        current_odom.y=current_odom.y + delta_y;
        current_odom.theta=current_odom.theta + delta_theta;
        current_odom.utime = current_encoders.utime;

        while(current_odom.theta >= 2*PI)
        {
            current_odom.theta = current_odom.theta - 2*PI;
        }
        while(current_odom.theta < 0)
        {
            current_odom.theta = current_odom.theta + 2*PI;
        }





        // get the current motor command state (if we have one)
        if (comms_get_topic_data(MBOT_MOTOR_COMMAND, &current_cmd))
        {
            int16_t l_cmd, r_cmd;                 // left and right motor commands
            float left_sp, right_sp;              // speed in m/s
            float measured_vel_l, measured_vel_r; // measured velocity in m/s
            float l_duty, r_duty;                 // duty cycle in range [-1, 1]
            float dt = MAIN_LOOP_PERIOD;          // time since last update in seconds
            uint64_t previousTimeStamp = 0;
            if (OPEN_LOOP)
            {
                /*************************************************************
                 * TODO:
                 *      - Implement the open loop motor controller to compute the left
                 *          and right wheel commands
                 *      - Determine the setpoint velocities for left and right motor using the wheel velocity model
                 *      - To compute the measured velocities, use dt as the timestep (∆t)
                 ************************************************************/

                
                double angularV = current_cmd.angular_v;
                double linearV = current_cmd.trans_v;
                
                left_sp = linearV - 0.5*angularV*WHEEL_BASE;
                right_sp = linearV + 0.5*angularV*WHEEL_BASE;

                
                l_duty = linearSpeedToDuty(left_sp,'l');
                r_duty = linearSpeedToDuty(right_sp,'r');

                

                // rc_motor_set(1, (int) l_duty);
                // rc_motor_set(3, (int) r_duty);

                if(current_cmd.utime != previousTimeStamp)
                {
                    // printf("currentTime: %d \n",current_cmd.utime);
                    // printf("left wheel speed:%f, right wheel speed:%f \n",left_sp,right_sp);
                    // printf("lduty:%f, rduty:%f \n\n\n",l_duty,r_duty);
                }
                else
                {
                    printf("same command, status will not change\n");
                }

                previousTimeStamp = current_cmd.utime;







                /*************************************************************
                
                 * End of TODO
                 *************************************************************/
            }
            else
            {
                /*************************************************************
                 * TODO:
                 *      - Implement the closed loop motor controller to compute the left
                 *          and right wheel commands
                 *      - To calculate the measured velocity, use MAIN_LOOP_PERIOD or latency_time
                 *          as the timestep
                 *      - Compute the error between the setpoint velocity and the measured velocity
                 *      - We recommend to use the open loop controller as part of the closed loop to improve
                 *          performance.
                 *          Example: open_loop_control(LEFT_MOTOR_CHANNEL, left_sp)
                 *      - Use the PID filters defined in mbot.h and main() function to calculate desired
                 *          duty, use rc_filter_march() function.
                 *          Example: rc_filter_march(&left_pid, left_error)
                 * TODO:
                 *      - Compute the error between the target and measured translation+rotation velocity
                 *      - Compute the new forward and rotation setpoints that will be used in
                 *          the wheel speed PID (these should be written in lines above
                 *          the previous controller)
                 *      - Update and the fwd_sp and turn_sp variables for this.
                 *
                 ************************************************************/
                float fwd_sp, turn_sp;                     // forward and turn setpoints in m/s and rad/s
                float measured_vel_fwd, measured_vel_turn; // measured forward and turn velocities in m/s and rad/s
                
                // following data are from encoder

                double angularV = current_cmd.angular_v;
                double linearV = current_cmd.trans_v;
                
                double lWheelTargetVelocity = linearV - 0.5*angularV*WHEEL_BASE;
                double rWheelTargetVelocity = linearV + 0.5*angularV*WHEEL_BASE;



                printf("angularV:%f,target left speed:%f,target right speed:%f\n",angularV,lWheelTargetVelocity,rWheelTargetVelocity);


                double timeFromLast = (current_encoders.utime - previous_encoders.utime)/1000000.0;
                double lWheelDistance = enc2meters * (current_encoders.left_delta);
                double rWheelDistance = enc2meters * (current_encoders.right_delta);
                
                double lWheelRealVelocity = lWheelDistance/timeFromLast;
                double rWheelRealVelocity = rWheelDistance/timeFromLast;

                // printf("%f\n",lWheelRealVelocity);

                double lWheelVelocityError = lWheelTargetVelocity - lWheelRealVelocity;
                double rWheelVelocityError = rWheelTargetVelocity - rWheelRealVelocity;

                float lWheelPidDeltaVel = rc_filter_march(&left_wheel_pid_filter, lWheelVelocityError);
                float rWheelPidDeltaVel = rc_filter_march(&right_wheel_pid_filter, rWheelVelocityError);
                
                l_duty = linearSpeedToDuty(lWheelPidDeltaVel,'l');
                r_duty = linearSpeedToDuty(rWheelPidDeltaVel,'r');

                /**
                 *  Example closed loop controller
                 *      (assuming the error between the target and measured is computed)
                 *
                 * float pid_delta_vel = rc_filter_march(&pid_filter, error);
                 * float desired_vel = commanded_val + pid_delta_vel;
                 */

                /*************************************************************
                 * End of TODO
                 *************************************************************/
                
            }
            // Clamp duty cycle to [-1, 1]
            l_duty = clamp_duty(l_duty);
            r_duty = clamp_duty(r_duty);

            // duty to motor command
            l_cmd = LEFT_MOTOR_POL * (int)(l_duty * 0.95 * pow(2, 15));
            r_cmd = RIGHT_MOTOR_POL * (int)(r_duty * 0.95 * pow(2, 15));


            // set left and right motor command
            rc_motor_set(LEFT_MOTOR_CHANNEL, l_cmd);
            rc_motor_set(RIGHT_MOTOR_CHANNEL, r_cmd);
        }

        // write the encoders to serial
        comms_write_topic(MBOT_ENCODERS, &current_encoders);
        // send odom on wire
        comms_write_topic(ODOMETRY, &current_odom);
        // write the IMU to serial
        comms_write_topic(MBOT_IMU, &current_imu);
        uint64_t fn_run_len = to_us_since_boot(get_absolute_time()) + timestamp_offset - cur_pico_time;
    }

    return true;
}

int main()
{
    bi_decl(bi_program_description("Firmware for the MBot Robot Control Board"));
    
    set_sys_clock_khz(125000, true); // set master clock to 250MHz (if problematic try 125Mhz)
    stdio_init_all(); // enable USB serial terminal
    sleep_ms(1500); // quick sleep so we can catch the bootup process in terminal
    printf("\nMBot Booting Up!\n");

    printf("initializinging motors...\n");
    rc_motor_init();
    printf("initializinging encoders...\n");
    rc_encoder_init();

    // Pins
    // for the i2c to the IMU
    const uint sda_pin = 4;
    const uint scl_pin = 5;

    // Ports
    i2c = i2c0;
    // Initialize I2C port at 400 kHz
    i2c_init(i2c, 400 * 1000);
    // Initialize I2C pins
    printf("setting i2c functions...\n");
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    printf("setting i2c pullups...\n");
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(sda_pin, scl_pin, GPIO_FUNC_I2C));

    printf("setting heartbeat LED GPIOs...\n");
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Initialize the IMU using the Digital Motion Processor
    printf("initializing DMP...\n");
    rc_mpu_config_t mpu_config = rc_mpu_default_config();
    mpu_config.i2c_bus = i2c;
    mpu_config.dmp_fetch_accel_gyro = 1;
    mpu_config.enable_magnetometer = 1;
    mpu_config.read_mag_after_callback = 0;
    mpu_config.orient = ORIENTATION_Z_UP;
    mpu_config.dmp_sample_rate = 200;
    
    // Calibrate the gyro to eliminate bias, Mbot must be still for this
    rc_mpu_calibrate_gyro_routine(mpu_config);
    sleep_ms(500);
    rc_mpu_initialize_dmp(&mpu_data, mpu_config);
    gpio_set_irq_enabled_with_callback(rc_MPU_INTERRUPT_GPIO, GPIO_IRQ_EDGE_FALL, true, &rc_dmp_callback);
    printf("MPU Initialized!\n");

    // create topics and register the serialize/deserialize functions
    printf("init comms...\r\n");
    comms_init_protocol();
    comms_init_topic_data();
    register_topics();

    // launch the other core's comm loop
    printf("starting comms on core 1...\r\n");
    multicore_launch_core1(comms_listener_loop);

    // wait for other core to get rolling
    sleep_ms(500);
    int running = 1;

    // run the main loop as a timed interrupt
    printf("starting the timed interrupt...\r\n");
    repeating_timer_t loop_timer;
    add_repeating_timer_ms(MAIN_LOOP_PERIOD * 1000, timer_cb, NULL, &loop_timer); // 1000x to convert to ms

    printf("Done Booting Up!\n\n");

    /*************************************************************
     * TODO:
     *  - wheel speed PID
     *      - Initilize the PID Filters rc_filter_empty()
     *      - Set the PID gains rc_filter_pid()
     * TODO:
     *  - body frame velocity PID
     *      - Initialize the PID filters for translation and rotation vel
     *      - Set the PID gains for translation and rotation vel
     *
     *************************************************************/

    // Example initialization of a PID filter defined in mbot.h
    left_wheel_pid_filter = rc_filter_empty();
    right_wheel_pid_filter = rc_filter_empty();

    // Example of assigning PID parameters (using pid_parameters_t from mbot.h)
    rc_filter_pid(&left_wheel_pid_filter,
                pid_params.kp,
                pid_params.ki,
                pid_params.kd,
                1.0 / pid_params.dFilterHz,
                1.0 / MAIN_LOOP_HZ);
    
    rc_filter_pid(&right_wheel_pid_filter,
                pid_params.kp,
                pid_params.ki,
                pid_params.kd,
                1.0 / pid_params.dFilterHz,
                1.0 / MAIN_LOOP_HZ);

    // Example of setting limits to the output of the filter
    // rc_filter_enable_saturation(&my_filter, min_val, max_val);

    /*************************************************************
     * End of TODO
     *************************************************************/

    if (OPEN_LOOP)
    {
        printf("Running in open loop mode\n");
    }
    else
    {
        printf("Running in closed loop mode\n");
    }

    while (running)
    {
        double timeFromLast = (current_encoders.utime - previous_encoders.utime)/1000000.0;
        double lWheelDistance = enc2meters * (current_encoders.left_delta);
        double rWheelDistance = enc2meters * (current_encoders.right_delta);
                
        double lWheelRealVelocity = lWheelDistance/timeFromLast;
        double rWheelRealVelocity = rWheelDistance/timeFromLast;

        // printf("\033[2A\r| lvelocity:\n\r %f",lWheelRealVelocity);
        
        // printf("\033[2A\r|      SENSORS      |           ODOMETRY          |     SETPOINTS     |\n\r|  L_Vel  |  R_Vel  |    X    |    Y    |    θ    |   FWD   |   ANG   \n\r|%7.3f  |%7.3f  |%7.3f  |%7.3f  |%7.3f  |%7.3f  |%7.3f  |", lWheelRealVelocity, rWheelRealVelocity, current_odom.x, current_odom.y, current_odom.theta, current_cmd.trans_v, current_cmd.angular_v);
    }
}

/**
 * @brief Clamp duty cycle between -1 and 1. If not applied, robot drives in reverse only
 *
 * @param duty
 */
float clamp_duty(float duty)
{
    if (duty > 1.0)
    {
        return 1.0;
    }
    else if (duty < -1.0)
    {
        return -1.0;
    }
    return duty;
}
