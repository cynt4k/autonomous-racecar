FROM lab/ros:base

MAINTAINER Tobias Schneider, me@cynt4k.de

RUN apt-get update && apt-get install -y apt-transport-https \
                                         ca-certificates \
                                         wget \
                                         gnupg \
                                         libnss3 \
                                         libxkbfile1 \
                                         libgconf-2-4 \
                                         libsecret-1-0 \
                                         libxss1 \
                                         libasound2 \
                                         --no-install-recommends && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y eclipse-cdt && \
    rm -fr /var/lib/apt/lists/*

RUN wget -O /tmp/vscode.deb https://projects.cdn.cynt4k.de/autonomous-racecar/vscode_arm64.deb && dpkg -i /tmp/vscode.deb && rm /tmp/vscode.deb

ADD scripts/ros_entrypoint.sh /ros_entrypoint.sh
