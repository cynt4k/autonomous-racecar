FROM lab/ros:base

ENV CATKIN_WS=/root/imu
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS/src

# Install catkin
RUN apt-get update && apt-get install -y gdb \
                                         gdbserver \
                                         python-pip \
                                         python-catkin-tools && \
    rm -fr /var/lib/apt/lists/*

# Hack to bash ;)
RUN mv /bin/sh /bin/sh-old && \
    ln -s /bin/bash /bin/sh

#ADD src $CATKIN_WS/src
RUN git clone https://github.com/KristofRobot/razor_imu_9dof.git --depth=1 $CATKIN_WS/src

# Install rosdeps
RUN apt-get update && rosdep install -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false && \
    rm -rf /var/lib/apt/lists/*

WORKDIR $CATKIN_WS
RUN source /ros_entrypoint.sh && \
    catkin build --no-status

# Add configs and launch files
ADD assets/my_razor.yaml $CATKIN_WS/src/config/
ADD assets/imu.launch $CATKIN_WS/src/launch/

RUN sed -i 's/# PLACEHOLDERSOURCE/source $CATKIN_WS\/devel\/setup.bash/g' /ros_entrypoint.sh

