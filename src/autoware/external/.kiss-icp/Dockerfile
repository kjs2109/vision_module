FROM ubuntu:jammy

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update -y && apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    sudo \
    wget \
    ninja-build \
    software-properties-common \
    python3 \
    python3-pip \
    ssh \
    gcc \
    g++ \
    gdb \
    rsync \
    tar \
    x11-utils \
    x11-apps \
    zip \
    locales \
    curl \
    gnupg2 \
    lsb-release && \
    apt-get clean


RUN locale-gen en_US en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LC_ALL en_US.UTF-8

RUN pip3 install --no-cache-dir open3d
RUN pip3 install --no-cache-dir kiss-icp

# ros2 humble 
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add -
RUN sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
RUN apt-get update && apt-get install -y ros-humble-desktop


RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool

# ROS 환경 자동 로드
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
ENV ROS_DISTRO humble


# ros2_ws 설정 & KISS-ICP ROS 2 Wrapper 빌드
RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws/src

# KISS-ICP ROS 2 Wrapper 소스코드 클론
RUN git clone https://github.com/PRBonn/kiss-icp.git

# 빌드
WORKDIR /root/ros2_ws
RUN source /opt/ros/humble/setup.bash && \
    colcon build

RUN echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]
