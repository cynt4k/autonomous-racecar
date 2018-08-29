#include "servo.hpp"

//! Constructor takes nothing
/*!
 * Init the params, set pca9685 frequency, set servo neutral positions and init node subscribers
 */
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
        servo->setAllPWM(0, 0);
        servo->setPWMFreq(50);
        sleep(1);
        servo->setPWM(servo_port, 0, SERVO_PWM_NEUTRAL);
        servo->setPWM(esc_port, 0, ESC_THROTTLE_NEUTRAL);

        sub_state = node.subscribe("servo/state", 100, &Servo::chatterServoState, this);
        sub_direction = node.subscribe("servo/direction", 100, &Servo::chatterDirectionPwm, this);
        sub_esc = node.subscribe("servo/esc", 100, &Servo::chatterEscPwm, this);

        ROS_INFO("Controller is ready...");
    }
}

Servo::~Servo() {
    delete servo;
}

//! Callback for the ServoState handler
/*!
 * \param servo_state servo_msgs/ServoState to do specific operations like instant stop
 */
void Servo::chatterServoState(const servo_msgs::ServoStateConstPtr &servo_state) {
    if (servo_state->servo_type == servo_state->STOP) {
        servo->setPWM(esc_port, 0, ESC_THROTTLE_NEUTRAL);
        ROS_INFO("ESC disabled");
    } else {
        ROS_INFO("Not implemented yet");
    }
}

//! Callback for the direction handler
/*!
 * \param pwm_state servo_msgs/PwmState to drive left or right. Valid ranges are between -1 and 1
 */
void Servo::chatterDirectionPwm(const servo_msgs::PwmStateConstPtr &pwm_state) {
    int value = SERVO_PWM_NEUTRAL;

    if (pwm_state->value < SERVO_LEFT_ANGLE || pwm_state->value > SERVO_RIGHT_ANGLE) {
        ROS_INFO("Invalid range");
        return;
    }

    if (pwm_state->value < 0) {
        value = mapRange((float) pwm_state->value, SERVO_LEFT_ANGLE, SERVO_NEUTRAL, SERVO_MAX_LEFT, SERVO_PWM_NEUTRAL);
    }
    if (pwm_state->value > 0) {
        value = mapRange((float) pwm_state->value, SERVO_NEUTRAL, SERVO_RIGHT_ANGLE, SERVO_PWM_NEUTRAL, SERVO_MAX_RIGHT);
    }

    servo->setPWM(servo_port, 0, value);


}

//! Callback for the esc handler
/*!
 * \param pwm_state servo_msgs/PwmState to throttle the esc. Valid ranges are between -1 and 1
 */

void Servo::chatterEscPwm(const servo_msgs::PwmStateConstPtr &pwm_state) {
    static int last_direction = ESC_NEUTRAL;
    int value = 0;

    if (pwm_state->value < ESC_MIN_THROTTLE || pwm_state->value > ESC_MAX_THROTTLE) {
        ROS_INFO("Invalid range");
        return;
    }

    if (pwm_state->value == 0.0) {
        value = ESC_THROTTLE_NEUTRAL;
    }
    if (pwm_state->value > 0) {
        value = mapRange((float) pwm_state->value, ESC_NEUTRAL, ESC_MAX_THROTTLE, ESC_THROTTLE_NEUTRAL, ESC_THROTTLE_FULL_FORWARD);
    }
    if (pwm_state->value < 0) {
        value = mapRange((float) pwm_state->value, ESC_MIN_THROTTLE, ESC_NEUTRAL, ESC_THROTTLE_FULL_REVERSE, ESC_THROTTLE_NEUTRAL);
    }

    if (last_direction < 0 && pwm_state->value > 0) {
        //TODO: Parse for IMU to detect stopping
        servo->setPWM(esc_port, 0, ESC_THROTTLE_NEUTRAL);
        sleep(1.5);
    }

    if (last_direction >= 0 && pwm_state->value < 0) {
        //TODO: Parse for IMU to detect stopping
        servo->setPWM(esc_port, 0, ESC_THROTTLE_NEUTRAL);
        sleep(2.5);

        // Hack to get into reverse mode
        servo->setPWM(esc_port, 0, value);
        sleep(0.2);
        servo->setPWM(esc_port, 0, ESC_THROTTLE_NEUTRAL);
    }

    last_direction = (pwm_state->value > 0) ? (int) ceil((float) pwm_state->value) : (int) floor((float) pwm_state->value);

    servo->setPWM(esc_port, 0, value);
}

//! Calculate linear mapping between two ranges of values
/*!
 * @param x reference value
 * @param x_min min value of the reference value
 * @param x_max max value of the reference value
 * @param y_min min value of the target value
 * @param y_max max value of the target value
 * @return new linear mapped value of x to y
 */

int Servo::mapRange(float x, int x_min, int x_max, int y_min, int y_max) {
    int  y = 0;
    float x_range, y_range, xy_ratio = 0;

    x_range = x_max - x_min;
    y_range = y_max - y_min;

    xy_ratio = x_range / y_range;

    y = (int) round(((x - x_min) / xy_ratio + y_min));
    return y;

}

//! Shutdown ros routine
void Servo::shutdown() {
    ros::shutdown();
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "servo");
    Servo s;
    ros::spin();
}