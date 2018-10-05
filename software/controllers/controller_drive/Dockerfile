FROM lab/ros:base

ENV CATKIN_WS=/root/controller_servo
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS/src

# Install catkin
RUN apt-get update && apt-get install -y python-pip \
                                         python-catkin-tools && \
    rm -fr /var/lib/apt/lists/*

# Hack to bash ;)
RUN mv /bin/sh /bin/sh-old && \
    ln -s /bin/bash /bin/sh

ADD src $CATKIN_WS/src

# Install rosdeps
RUN echo "yaml file:///$CATKIN_WS/src/rosdep.yaml" >> /etc/ros/rosdep/sources.list.d/40-rosdep.list && \
    apt-get update && rosdep update && \
    rosdep install -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false && \
    rm -rf /var/lib/apt/lists/*

WORKDIR $CATKIN_WS
RUN source /ros_entrypoint.sh && \
    catkin build --no-status

RUN sed -i 's/# PLACEHOLDERSOURCE/source $CATKIN_WS\/devel\/setup.bash/g' /ros_entrypoint.sh

