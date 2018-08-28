#include "servo.hpp"

Servo::Servo() {
    ros::NodeHandle pnode("~");
    pnode.param("bus_id", bus_id, 0);
    pnode.param("address", address, 0x40);

    if (!pnode.hasParam("servo_port")) {
        ROS_ERROR("Servo port not defined");
        shutdown();
    } else if (!pnode.hasParam("esc_port")) {
        ROS_ERROR("ESC port not defined");
        shutdown();
    } else {
        pnode.getParam("servo_port", servo_port);
        pnode.getParam("esc_port", esc_port);
        servo = new PCA9685(bus_id, address);
        servo->setPWMFreq(50);
        sleep(1);
        servo->setPWM(esc_port, 0, ESC_THROTTLE_NEUTRAL);

        sub_state = node.subscribe("servo/state", 100, &Servo::chatterServoState, this);
        sub_direction = node.subscribe("servo/direction", 100, &Servo::chatterDirectionPwm, this);
        sub_esc = node.subscribe("servo/esc", 100, &Servo::chatterEscPwm, this);

        ROS_INFO("Controller is ready...");
    }
}

void Servo::chatterServoState(const servo_msgs::ServoStateConstPtr &servo_state) {
    if (servo_state->servo_type == servo_state->STOP) {
        servo->setPWM(esc_port, 0, ESC_THROTTLE_NEUTRAL);
        ROS_INFO("ESC disabled");
    } else {
        ROS_INFO("Not implemented yet");
    }
}

void Servo::chatterDirectionPwm(const servo_msgs::PwmStateConstPtr &pwm_state) {
    if (pwm_state->value < SERVO_MIN_PULSE || pwm_state->value > SERVO_MAX_PULSE) {
        ROS_INFO("Invalid range");
    } else {
        servo->setPWM(servo_port, 0, (int) pwm_state->value);
    }
}

void Servo::chatterEscPwm(const servo_msgs::PwmStateConstPtr &pwm_state) {
    if (pwm_state->value < ESC_THROTTLE_FULL_REVERSE || pwm_state->value > ESC_THROTTLE_FULL_FORWARD) {
        ROS_INFO("Invalid range");
    } else {
        servo->setPWM(esc_port, 0, (int) pwm_state->value);
    }
}

void Servo::shutdown() {
    ros::shutdown();
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "servo");
    Servo s;
    ros::spin();
}