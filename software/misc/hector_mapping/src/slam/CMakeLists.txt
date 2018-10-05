cmake_minimum_required(VERSION 2.8.3)
project(hector_mapping)

find_package(
        catkin REQUIRED COMPONENTS
        roslaunch
)

catkin_package()

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
