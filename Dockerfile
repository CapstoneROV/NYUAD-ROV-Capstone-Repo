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
    ros-melodic-geodesy \
    ros-melodic-robot-localization \
    ros-melodic-message-to-tf

#Install Octomap package 
RUN sudo apt-get install --no-install-recommends -y \
    ros-melodic-octomap \
    ros-melodic-octomap-ros \
    ros-melodic-octomap-msgs
    

# Install Mavros
RUN sudo apt-get install -y \
    ros-melodic-mavros \
    ros-melodic-mavros-extras \
    ros-melodic-mavros-msgs

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
    pymavlink 

RUN pip3 install --no-cache-dir \
    numpy \
    scipy \
    matplotlib \
    rospkg \
    pandas \
    pyyaml \
    empy==3.3.4 \
    pexpect \
    future

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

# Checkout specific commit (right before python3 transition)
# RUN git checkout f823848
RUN git checkout 8ae34a1

# Instructions from http://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html
RUN git submodule update --init --recursive

# Avoid prompt for timezone in tzdata
RUN sudo apt-get install -y tzdata

# Install ArduPilot SITL dependencies & build
ENV SKIP_AP_EXT_ENV=1 SKIP_AP_GRAPHIC_ENV=1 SKIP_AP_COV_ENV=1 SKIP_AP_GIT_CHECK=1
RUN ./Tools/environment_install/install-prereqs-ubuntu.sh -y
# RUN pip3 install pexpect future
RUN ./waf distclean && \
    ./waf configure --board sitl && \
    ./waf sub

RUN sudo chmod +x $HOME/ardupilot/Tools/autotest/sim_vehicle.py

# Install geographic lib dataset
RUN sudo curl -sL https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh -o install_geographiclib_datasets.sh
RUN sudo chmod +x install_geographiclib_datasets.sh
RUN sudo bash ./install_geographiclib_datasets.sh

# COPY capstonerov $HOME/capstonerov
COPY . $HOME/capstonerov
WORKDIR $HOME/capstonerov
RUN sudo chown -R $USER:$USER $HOME/capstonerov

# Source the ROS environment by default
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN echo 'export PATH="$PATH:$HOME/.local/bin"' >> ~/.bashrc

# Source the Gazebo environment for plugins
RUN /bin/bash -c "source /usr/share/gazebo/setup.sh"
RUN echo "export GAZEBO_MODEL_PATH=$HOME/ardupilot_gazebo/models:${GAZEBO_MODEL_PATH}" >> ~/.bashrc
RUN echo "export GAZEBO_MODEL_PATH=$HOME/ardupilot_gazebo/models_gazebo:${GAZEBO_MODEL_PATH}" >> ~/.bashrc
RUN echo "export GAZEBO_RESOURCE_PATH=$HOME/ardupilot_gazebo/worlds:${GAZEBO_RESOURCE_PATH}" >> ~/.bashrc
RUN echo "export GAZEBO_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GAZEBO_PLUGIN_PATH}" >> ~/.bashrc

# Make workspace
RUN pip install empy==3.3.4 mavproxy==1.8.69
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && catkin_make -j"
RUN echo "source $HOME/capstonerov/devel/setup.bash" >> ~/.bashrc

WORKDIR $HOME/capstonerov/include/ardupilot_gazebo
RUN make build 
# RUN mkdir build && cd ./build && cmake $HOME/capstonerov/include/ardupilot_gazebo && make -j && sudo make install

WORKDIR $HOME/capstonerov
# Set the default command to bash
CMD exec bash
