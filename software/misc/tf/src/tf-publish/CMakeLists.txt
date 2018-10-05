cmake_minimum_required(VERSION 2.8.3)
project(tf-publish)

find_package(
        catkin REQUIRED COMPONENTS
        nav_msgs
        roslaunch
)

catkin_package()

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
