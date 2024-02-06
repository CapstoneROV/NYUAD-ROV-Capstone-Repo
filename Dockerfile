# Use Ubuntu 18.04 as base image
FROM ubuntu:18.04

# Install Python 2.7 and Python 3
RUN apt-get update && \
    apt-get install -y python2.7 python3 python-pip python3-pip

# Add the ROS repository
RUN apt-get install -y curl gnupg2 lsb-release && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Install ROS Melodic
RUN apt-get update && \
    apt-get install -y ros-melodic-desktop-full

# Install additional ROS packages
RUN apt-get install --no-install-recommends -y \
    ros-melodic-uuv-simulator

# Install rosdep
RUN apt-get install -y python-rosdep

# Initialize rosdep
RUN rosdep init && \
    rosdep update

# Install common Python packages for robotics and ROS
RUN pip install --no-cache-dir \
    numpy \
    scipy \
    matplotlib \
    pandas \
    rospkg \
    catkin_pkg \
    pyyaml \
    empy

RUN pip3 install --no-cache-dir \
    numpy \
    scipy \
    matplotlib \
    pandas \
    rospkg \
    catkin_pkg \
    pyyaml \
    empy

# Install additional packages for GUI x 11 forwarding
RUN apt-get update && apt-get install -y \
    x11-apps \
    x11-xserver-utils \
    libgl1-mesa-glx \
    mesa-utils

# Install Git
RUN apt-get install -y git; git config --global url."https://github.com/".insteadOf git://github.com/

# SETUP ARDUSUB SITL 
# Clone ArduPilot repository
RUN git clone --no-checkout https://github.com/ArduPilot/ardupilot.git ardupilot
WORKDIR /ardupilot

# Checkout specific commit
RUN git checkout f823848

# Instructions from http://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html
RUN git submodule update --init --recursive

# Avoid prompt for timezone in tzdata
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get install -y sudo lsb-release tzdata
ENV DEBIAN_FRONTEND=

# Install ArduPilot SITL dependencies & build
RUN USER=nobody Tools/environment_install/install-prereqs-ubuntu.sh -y
RUN ./waf distclean && \
    ./waf configure --board sitl && \
    ./waf sub

# TODO: MAKE sim_vehicle.py executable & install QGroundControl

# Source the ROS environment by default
RUN echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc

# Set the working directory to the mounted volume
WORKDIR /capstonerov

# Set the default command
CMD ["bash"]
