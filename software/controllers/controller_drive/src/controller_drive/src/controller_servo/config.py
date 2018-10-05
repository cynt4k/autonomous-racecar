import os

# Debug
DEBUG = False
DEBUG_IP = '127.0.0.1'
DEBUG_PORT = 1234
# Name
DEFAULT_NAME = 'controller_servo'

# Topics
TOPIC_DEFAULT_ESC = '/servo/esc'
TOPIC_DEFAULT_DIRECTION = '/servo/direction'
TOPIC_DEFAULT_JOYSTICK = '/joy'

# Buttons
JOYSTICK_BUTTON_DEAD_MAN = 2
JOYSTICK_BUTTON_RECORD = 0
JOYSTICK_BUTTON_OPERATING_TYPE = 7

# Axis
JOYSTICK_AXIS_LEFT_RIGHT = 0
JOYSTICK_AXIS_THROTTLE_REVERSE = 4

# Limits
LIMIT_DIRECTION_LEFT = -1
LIMIT_DIRECTION_RIGHT = 1
LIMIT_ESC_THROTTLE = 1
LIMIT_ESC_REVERSE = -1
STOP_TOLERANCE = 0.08

# Operating mode
OPERATING_MODE_AUTONOM = 1
OPERATING_MODE_MANUAL = 0

# Button
BUTTON_PRESSED = 1
BUTTON_RELEASED = 0

# Dead man limit
LIMIT_DEAD_MAN_RELEASED = -0.5
LIMIT_DEAD_MAN_PRESSED = -1
