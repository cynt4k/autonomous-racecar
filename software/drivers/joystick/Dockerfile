FROM lab/ros:base

ENV CATKIN_WS=/root/joystick
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS/src

# Install catkin
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-joy && \
    rm -fr /var/lib/apt/lists/*

# Hack to bash ;)
RUN mv /bin/sh /bin/sh-old && \
    ln -s /bin/bash /bin/sh

WORKDIR $CATKIN_WS
