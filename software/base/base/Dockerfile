FROM ros:kinetic-ros-base

MAINTAINER Tobias Schneider, me@cynt4k.de

# Fix Dialog problem

ENV DEBIAN_FRONTEND noninteractive

# Set time zone
RUN echo Europe/Berlin > /etc/timezone && \
    rm /etc/localtime && \
    ln -s /usr/share/zoneinfo/Europe/Berlin /etc/localtime && \
    dpkg-reconfigure --frontend noninteractive tzdata

# Install basic packages
RUN apt-get update && apt-get -y install vim \
                                         less

ADD scripts/ros_entrypoint.sh /ros_entrypoint.sh
