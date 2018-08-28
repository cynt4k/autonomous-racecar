cmake_minimum_required(VERSION 2.8.3)
project(servo)

set(ROS_BUILD_TYPE Debug)

find_package(
        catkin REQUIRED COMPONENTS
        roslaunch
        roscpp
        servo_msgs
)

catkin_package()

include_directories(
        src/lib/I2C src/lib/PCA9685 ${catkin_INCLUDE_DIRS}
)

add_library(I2C src/lib/I2C/I2C.cpp)
target_link_libraries(I2C ${catkin_LIBRARIES})

add_library(PCA9685 src/lib/PCA9685/PCA9685.cpp)
target_link_libraries(PCA9685 I2C ${catkin_LIBRARIES})

add_executable(servo_run src/servo.cpp)
target_link_libraries(servo_run PCA9685 I2C ${catkin_LIBRARIES})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
