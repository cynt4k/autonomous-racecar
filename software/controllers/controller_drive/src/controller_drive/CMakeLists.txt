cmake_minimum_required(VERSION 2.8.3)
project(controller_servo)

find_package(catkin REQUIRED COMPONENTS roslaunch servo_msgs)

catkin_python_setup()

catkin_package()


install(PROGRAMS scripts/controller
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
