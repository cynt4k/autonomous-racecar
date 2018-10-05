FROM lab/ros:base

MAINTAINER Tobias Schneider, me@cynt4k.de


RUN apt-get update && apt-get install -y curl \
                                      gdb \
                                      gdb-multiarch \
                                      ros-kinetic-desktop=1.3.2-0* && \
    rm -rf /var/lib/apt/lists/* && \
    rm -rf /tmp/*


ENV CLION_VERSION=2018.1

WORKDIR /opt
RUN curl -L https://download.jetbrains.com/cpp/CLion-{$CLION_VERSION}.tar.gz | tar xvz

ENV PYCHARM_VERSION=2018.1
RUN curl -L https://download-cf.jetbrains.com/python/pycharm-professional-${PYCHARM_VERSION}.tar.gz | tar xvz

CMD /bin/bash
