cmake_minimum_required(VERSION 2.8.3)
project(servo_msgs)

find_package(
        catkin REQUIRED COMPONENTS
        message_generation
        std_msgs
)

add_message_files(
        FILES
        ServoState.msg
        PwmState.msg
)

generate_messages(
        DEPENDENCIES std_msgs
)

catkin_package(
        CATKIN_DEPENDS
        message_runtime
)