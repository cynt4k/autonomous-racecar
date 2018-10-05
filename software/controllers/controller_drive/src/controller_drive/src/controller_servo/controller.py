import rospy
import config
from joystick import *


def init():
    direction_topic = rospy.get_param('~topic_direction', config.TOPIC_DEFAULT_DIRECTION)
    esc_topic = rospy.get_param('~topic_esc', config.TOPIC_DEFAULT_ESC)
    joystick_topic = rospy.get_param('~topic_joystick', config.TOPIC_DEFAULT_JOYSTICK)

    rospy.init_node(rospy.get_param('~name', config.DEFAULT_NAME))

    Joystick.init_subscribers(joystick_topic, direction_topic, esc_topic)
    rospy.spin()
