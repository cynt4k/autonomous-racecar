FROM lab/ros:base

ENV CATKIN_WS=/root/hector_mapping
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS/src

# Install catkin
RUN apt-get update && apt-get install -y ros-kinetic-hector-mapping \
                                         python-pip \
                                         python-catkin-tools && \
    rm -fr /var/lib/apt/lists/*

# Hack to bash ;)
RUN mv /bin/sh /bin/sh-old && \
    ln -s /bin/bash /bin/sh

ADD assets/hector_mapping.launch $CATKIN_WS/src
RUN source /ros_entrypoint.sh && roscd hector_mapping && mv $CATKIN_WS/src/hector_mapping.launch launch/

# RUN sed -i 's/# PLACEHOLDERSOURCE/source $CATKIN_WS\/devel\/setup.bash/g' /ros_entrypoint.sh

