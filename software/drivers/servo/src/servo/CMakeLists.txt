cmake_minimum_required(VERSION 2.8.3)
project(servo)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	servo_msgs
)

include_directories(
	src/lib/PCA9685/src src/lib/I2C/src ${CATKIN_INCLUDE_DIRS}
)

add_library(I2C src/lib/I2C/src/I2C.cpp)
target_link_libraries(I2C ${CATKIN_LIBRARIES})

add_library(PCA9685 src/lib/PCA9685/src/PCA9685.cpp)
target_link_libraries(I2C PCA9685 ${CATKIN_LIBRARIES})

add_executable(servo src/servo.cpp)
target_link_libraries(servo I2C PCA9685 ${CATKIN_LIBRARIES})
