# Use Ubuntu 18.04 as base image
FROM ubuntu:18.04

# To avoid prompts in setup
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install --no-install-recommends -y \
    lsb-release \
    curl \
    gnupg2 \
    sudo \
    software-properties-common

# Add the ROS repository
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Add a non-root user for pip & ardusub
ENV USER=ardupilot
RUN useradd -U -m $USER && \
    usermod -G users $USER

ENV HOME=/home/ardupilot
RUN echo "ardupilot ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USER
RUN echo 'Defaults env_keep += "DEBIAN_FRONTEND USER"' | EDITOR='tee -a' visudo
RUN chmod 0440 /etc/sudoers.d/$USER
USER $USER

# Install ROS Melodic
RUN sudo apt-get update && sudo apt-get install -y ros-melodic-desktop-full

# Install Python 2.7 and Python 3
RUN sudo apt-get install -y python2.7 python3 python-pip python3-pip

# Install additional ROS packages
RUN sudo apt-get install --no-install-recommends -y \
    ros-melodic-uuv-simulator \
    catkin_tools \
    ros-melodic-geodesy \
    ros-melodic-robot-localization

# Install Mavros
RUN sudo apt-get install -y \
    ros-melodic-mavros \
    ros-melodic-mavros-extras \
    ros-melodic-mavros-msgs

# Install geographic lib dataset
RUN curl -
RUN sudo ./install_geographiclib_datasets.sh

# Install rosdep
RUN sudo apt-get install -y python-rosdep

# Initialize rosdep
RUN sudo rosdep init && \
    rosdep update && \
    sudo rosdep fix-permissions

# Install common Python packages for robotics and ROS
RUN pip install --no-cache-dir \
    numpy \
    scipy \
    matplotlib \
    pandas \
    rospkg \
    catkin_pkg \
    pyyaml \
    empy==3.3.4 \
    mavproxy \
    pymavlink 

RUN pip3 install --no-cache-dir \
    numpy \
    scipy \
    matplotlib \
    rospkg \
    pandas \
    pyyaml \
    empy==3.3.4

# Install additional packages for GUI x 11 forwarding
RUN sudo apt-get update && sudo apt-get install -y \
    x11-apps \
    x11-xserver-utils \
    libgl1-mesa-glx \
    mesa-utils

# Install Git
RUN sudo apt-get install -y git; git config --global url."https://github.com/".insteadOf git://github.com/

# SETUP ARDUSUB SITL 
# Clone ArduPilot repository
WORKDIR $HOME
RUN sudo git clone --no-checkout https://github.com/ArduPilot/ardupilot.git ardupilot

# Change ownership of the ardupilot directory to the non-root user
RUN sudo chown -R $USER:$USER $HOME/ardupilot
WORKDIR $HOME/ardupilot

# Checkout specific commit
RUN git checkout f823848

# Instructions from http://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html
RUN git submodule update --init --recursive

# Avoid prompt for timezone in tzdata
RUN sudo apt-get install -y tzdata

# Install ArduPilot SITL dependencies & build
ENV SKIP_AP_EXT_ENV=1 SKIP_AP_GRAPHIC_ENV=1 SKIP_AP_COV_ENV=1 SKIP_AP_GIT_CHECK=1
RUN ./Tools/environment_install/install-prereqs-ubuntu.sh -y
RUN ./waf distclean && \
    ./waf configure --board sitl && \
    ./waf sub

# Install QGroundControl
# RUN usermod -a -G dialout $USER
RUN sudo apt-get remove modemmanager -y
RUN sudo apt install -y \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gstreamer1.0-gl \
    libqt5gui5 \
    libfuse2
RUN sudo curl -fsSL https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage -o QGroundControl.AppImage
RUN sudo chmod +x QGroundControl.AppImage
RUN sudo chmod +x $HOME/ardupilot/Tools/autotest/sim_vehicle.py

# Set the working directory to the mounted volume
WORKDIR $HOME/capstonerov
RUN sudo chown -R $USER:$USER $HOME/capstonerov
RUN sudo apt-get install -y ros-melodic-robot-localization
# RUN sudo chmod -R 777 $HOME

# Source the ROS environment by default
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN echo 'export PATH="$PATH:$HOME/.local/bin"' >> ~/.bashrc

# # Create catkin workspace
# RUN mkdir -p ~/catkin_ws/src
# RUN sudo chmod -R +777 ~/catkin_ws

# Just for tonight's hack, make the copy automated 
# Set the default command
CMD mkdir ~/permission/ && cp -r ~/capstonerov/ ~/permission/ && sudo chmod -R 777 ./ && cd ~/permission/capstonerov && exec bash
