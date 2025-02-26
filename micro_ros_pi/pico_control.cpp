#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <vector>

#include <std_msgs/msg/float64.h>
#include <geometry_msgs/msg/vector3.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/joy.h>

#include <rmw_microros/rmw_microros.h>

#include <hardware/pio.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"

#include "pico_quadrature_encoder/include/QuadratureEncoder.hpp"
#include "motor/src/motor.c"            //TODO: Figure out why cmake fails with .h but works with .c include
#include "pid_controller/src/pid.c"

const uint LED_PIN = 25;

//publishers
rcl_publisher_t publisherEncoder;
rcl_publisher_t publisherTimer;
rcl_publisher_t publisherWheelVel;

//subscribe to /cmd_vel
rcl_subscription_t velocitySubscriber;

geometry_msgs__msg__Twist cmd_velocities;
geometry_msgs__msg__Vector3 encoderMSG;
geometry_msgs__msg__Vector3 wheelVelMSG;
std_msgs__msg__Float64 timerMSG;

std::vector<QuadratureEncoder> encoders = {QuadratureEncoder(16, 0, 48, 75), QuadratureEncoder(14, 1, 48, 75), QuadratureEncoder(4, 2, 48, 75)};

//characteristics for motors 1,2,3
float motorForwardSlopes[3] = {5.72, 5.1, 5.61};
float motorForwardIntercepts[3] = {39, 37.1, 35.6};
float motorBackwardSlopes[3] = {5.53, 5.06, 5.31};
float motorBackwardIntercepts[3] = {39.6, 35.6, 37.3};

//current target vel (rad/s)
float motorTarget[3] = {0,0,0};
float motorDuty[3] = {0,0,0};

int motorPINS[3][3] = {{18,19,20}, {11,12,13}, {6,7,8}};

//initalize motors
float motorWrap[3] = {(float)initMotor(motorPINS[0], 10000, motorDuty[0]), (float)initMotor(motorPINS[1], 10000, motorDuty[1]), (float)initMotor(motorPINS[2], 10000, motorDuty[2])};

//global velocity commands from /cmd_vel
float bot_V = 0; // m/s
float bot_Vn = 0; // m/s
float bot_theta = 0; // rad/s


//robot params
const float wheelRAD = .048; //  m
const float wheelANGLE = 30 * (M_PI / 180); // rad 


struct PIDController pid[3] = { {1.7, .5, 0, 0, 0}, 
                      {1.7, .5, .15, 0, 0},
                      {1.7, .5, .15, 0, 0} };



void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    //find time since last timer_callback
    float sampling_time = (float)last_call_time / 1000000000;

    //for publishing last sampling time
    timerMSG.data = sampling_time;

    //update encoder readings for next velocity updates
    encoders[0].update(sampling_time);
    encoders[1].update(sampling_time);
    encoders[2].update(sampling_time);
    
    //store for publishing encoder values to topic
    encoderMSG.x = (double)encoders[0].get_count();
    encoderMSG.y = (double)encoders[1].get_count();
    encoderMSG.z = (double)encoders[2].get_count();

    //calculate individual motor velocities from inverse kinematic formulas in meters/sec and then convert to rad/sec
    float v_m1 = -bot_V * cos(wheelANGLE) + bot_Vn * sin(wheelANGLE) + bot_theta * .105;
    float v_m2 = bot_V * cos(wheelANGLE) + bot_Vn * sin(wheelANGLE) + bot_theta * .105;
    float v_m3 = -bot_Vn + bot_theta * .105;

    motorTarget[0] = v_m1 / .048;
    motorTarget[1] = v_m2 / .048;
    motorTarget[2] = v_m3 / .048;

    //individually update motor duty/speed
   for(int cnt = 0; cnt < 3; cnt++){
        //pid controller
        float new_speed = motorTarget[cnt] + PID_step(&pid[cnt], motorTarget[cnt], encoders[cnt].get_velocity()) * sampling_time;

        //set speed using appropriate motor slopes/intercepts for forwards and backwards movement 
        if(new_speed > 0){
            motorDuty[cnt] = new_speed * motorForwardSlopes[cnt] + motorForwardIntercepts[cnt];
        } else {
            motorDuty[cnt] = new_speed * motorBackwardSlopes[cnt] - motorBackwardIntercepts[cnt];
        }

        //clamp motor duty between [-99,99]
        motorDuty[cnt] = std::max(std::min(motorDuty[cnt], (float)99), (float)-99);
        changeDuty(motorWrap[cnt], motorDuty[cnt], motorPINS[cnt]);
    }

    wheelVelMSG.x = encoders[0].get_velocity();
    wheelVelMSG.x = encoders[1].get_velocity();
    wheelVelMSG.x = encoders[2].get_velocity();
   
    rcl_publish(&publisherWheelVel, &wheelVelMSG, NULL);
    rcl_publish(&publisherEncoder, &encoderMSG, NULL);
    rcl_publish(&publisherTimer, &timerMSG, NULL);
}

void cmd_vel_callback(const void * msgin)
{
  // Cast received message to used type
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  // Process message
  bot_V = cmd_velocities.linear.x;
  bot_Vn = cmd_velocities.linear.y;
  bot_theta = cmd_velocities.angular.z;
}

int main()
{
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico_node", "", &support);

    rclc_publisher_init_default(
        &publisherEncoder,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
        "encoder_publisher");

    rclc_publisher_init_default(
        &publisherEncoder,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
        "wheel_vel_publisher");


    rclc_publisher_init_default(
        &publisherTimer,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
        "timer_publisher");

    rclc_subscription_init_default(
        &velocitySubscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel");


    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(50),
        timer_callback);

    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_timer(&executor, &timer);
    rcl_ret_t rc = rclc_executor_add_subscription(&executor, &velocitySubscriber, &cmd_velocities, &cmd_vel_callback, ON_NEW_DATA);

    
    if (RCL_RET_OK != rc) {
        return -1;
    }
    

    gpio_put(LED_PIN, 1);

    timerMSG.data = 0;
    
    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}