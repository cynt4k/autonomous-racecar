#ifndef SERVO_HPP_
#define SERVO_HPP_

#include <cmath>

#include <ros/ros.h>
#include <servo_msgs/ServoState.h>
#include <servo_msgs/PwmState.h>

#include "PCA9685.h"

#define SERVO_MAX_LEFT 256
#define SERVO_PWM_NEUTRAL 384
#define SERVO_MAX_RIGHT 450

#define SERVO_MIN_PULSE SERVO_MAX_RIGHT
#define SERVO_MAX_PULSE SERVO_MAX_LEFT
#define SERVO_LEFT_ANGLE -1
#define SERVO_NEUTRAL 0
#define SERVO_RIGHT_ANGLE 1

#define ESC_THROTTLE_FULL_REVERSE 204
#define ESC_THROTTLE_NEUTRAL 307
#define ESC_THROTTLE_FULL_FORWARD 409
#define ESC_MIN_THROTTLE -1
#define ESC_NEUTRAL 0
#define ESC_MAX_THROTTLE 1

//! Main class that for the Servo driver
class Servo {
public:
    Servo();

    virtual ~Servo();

private:
    ros::NodeHandle node;
    ros::NodeHandle pnode;
    PCA9685 *servo;

    int bus_id, address, servo_port, esc_port;

    ros::Subscriber sub_state;
    ros::Subscriber sub_direction;
    ros::Subscriber sub_esc;

    void chatterServoState(const servo_msgs::ServoStateConstPtr &servo_state);
    void chatterDirectionPwm(const servo_msgs::PwmStateConstPtr &pwm_state);
    void chatterEscPwm(const servo_msgs::PwmStateConstPtr &pwm_state);

    int mapRange(float x, int x_min, int x_max, int y_min, int y_max);

    void shutdown();
};

#endif /*SERVO_HPP_ */
