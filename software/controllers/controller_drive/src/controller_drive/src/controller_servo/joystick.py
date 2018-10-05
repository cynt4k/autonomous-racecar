import rospy
import config

from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from servo_msgs.msg import PwmState


class Joystick:

    def __init__(self, pub_direction, pub_esc, pub_record=None, pub_operating_mode=None):
        self.pub_operating_mode = pub_operating_mode
        self.pub_record = pub_record
        self.pub_esc = pub_esc
        self.pub_direction = pub_direction

        self.dead_man_switch = 0
        self.operating_mode = config.OPERATING_MODE_MANUAL

        # Static variables
        self.last_state_operating_mode = config.BUTTON_RELEASED

    @staticmethod
    def init_subscribers(joystick_topic, direction_topic, esc_topic):
        pub_record = rospy.Publisher('record', Bool, queue_size=1)
        pub_operating_mode = rospy.Publisher('operation_mode', Bool, queue_size=1)
        pub_esc = rospy.Publisher(esc_topic, PwmState, queue_size=20)
        pub_direction = rospy.Publisher(direction_topic, PwmState, queue_size=20)

        joy = Joystick(pub_direction, pub_esc, pub_record, pub_operating_mode)

        rospy.Subscriber('joy', Joy, joy._handle_joystick)

    def _handle_joystick(self, data):
        msg_direction = PwmState()
        msg_esc = PwmState()

        axes = list(data.axes)
        buttons = list(data.buttons)

        if config.LIMIT_DEAD_MAN_PRESSED <= axes[config.JOYSTICK_BUTTON_DEAD_MAN] <= config.LIMIT_DEAD_MAN_RELEASED:

            if self.last_state_operating_mode is config.BUTTON_RELEASED:
                if buttons[config.JOYSTICK_BUTTON_OPERATING_TYPE] is config.BUTTON_PRESSED:
                    self.operating_mode = not self.operating_mode

            self.last_state_operating_mode = buttons[config.JOYSTICK_BUTTON_OPERATING_TYPE]

            if self.operating_mode is config.OPERATING_MODE_MANUAL:
                if 2 <= config.JOYSTICK_AXIS_LEFT_RIGHT <= 3:
                    axes[config.JOYSTICK_AXIS_LEFT_RIGHT] = axes[config.JOYSTICK_AXIS_LEFT_RIGHT] * -1

                if 2 <= config.JOYSTICK_AXIS_THROTTLE_REVERSE <= 3:
                    axes[config.JOYSTICK_AXIS_THROTTLE_REVERSE] = axes[config.JOYSTICK_AXIS_THROTTLE_REVERSE] * -1

                if -config.STOP_TOLERANCE <= axes[config.JOYSTICK_AXIS_THROTTLE_REVERSE] < config.STOP_TOLERANCE:
                    axes[config.JOYSTICK_AXIS_THROTTLE_REVERSE] = 0.0

                msg_direction.value = axes[config.JOYSTICK_AXIS_LEFT_RIGHT]
                msg_esc.value = axes[config.JOYSTICK_AXIS_THROTTLE_REVERSE]

                self.pub_direction.publish(msg_direction)
                self.pub_esc.publish(msg_esc)
        else:
            msg_direction.value = 0.0
            msg_esc.value = 0.0

            self.pub_direction.publish(msg_direction)
            self.pub_esc.publish(msg_esc)

        return
