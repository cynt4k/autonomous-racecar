#ifndef SERVO_HPP_
#define SERVO_HPP_


#include <ros/ros.h>
#include <servo_msgs/ServoState.h>
#include <servo_msgs/PwmState.h>

#include "PCA9685.h"

#define SERVO_MIN_PULSE 150
#define SERVO_MAX_PULSE 600
#define SERVO_PULSE_RANGE 4096

#define ESC_THROTTLE_FULL_REVERSE 204
#define ESC_THROTTLE_NEUTRAL 307
#define ESC_THROTTLE_FULL_FORWARD 409

class Servo {
public:
    Servo();

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

    void shutdown();
};

#endif /*SERVO_HPP_ */