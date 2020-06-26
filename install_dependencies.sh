#!/bin/bash
set -e
# Install the dependencies for GymFC on Ubuntu 18.04. 
# Requires sudo

GAZEBO_MAJOR_VERSION=10 
GAZEBO_VERSION=gazebo10_10.1.0
# Running sudo on this script will give $USER as root so get the currently logged in user instead
USERNAME=$(logname)
DART_VERSION=v6.7.0
ROS_DISTRO=dummy
# Compiling Gazebo can be very memory intensive, this variable passes additional flags to make for dart and gazebo. 
# By default this sets the number of parallel jobs to 1. If you set this too high make will crash with out of memory errors. 
# If you have sufficient memory, increase this value for a faster install.
MAKE_FLAGS=${MAKE_FLAGS:=-j1}

# Remove any other installations of Dart installed through debian packages
apt-get remove -y libdart*

# Install Dart dependencies
apt-get update && apt-get -y install \
    build-essential \
    cmake \
    pkg-config \
    git \
    libeigen3-dev \
    libassimp-dev \
    libccd-dev \
    libfcl-dev \
    libboost-regex-dev \
    libboost-system-dev

# Octomap is specified as an optional dependency but building is not linking without it, http://dartsim.github.io/install_dart_on_ubuntu.html
apt-get update && apt-get -y install \
    liboctomap-dev

# Build Dart
git clone https://github.com/dartsim/dart.git /tmp/dart \
    && cd /tmp/dart && git checkout $DART_VERSION \
    && mkdir build && cd build \
    && cmake .. \
    && make $MAKE_FLAGS \
    && make install 

# Install Gazebo from source following the instructions from here: http://gazebosim.org/tutorials?tut=install_from_source&cat=install

# Prepare Gazebo
apt-get -y remove '.*gazebo.*' '.*sdformat.*' '.*ignition-math.*' '.*ignition-msgs.*' '.*ignition-transport.*'

apt-get update && apt-get -y install \
    lsb-release \
    wget \
    mercurial \
    libboost-all-dev

sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
    && wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - && apt-get update 


# Install Gazebo dependencies
wget https://raw.githubusercontent.com/ignition-tooling/release-tools/master/jenkins-scripts/lib/dependencies_archive.sh -O /tmp/dependencies.sh
. /tmp/dependencies.sh
echo ${BASE_DEPENDENCIES} ${GAZEBO_BASE_DEPENDENCIES} ${DART_DEPENDENCIES} | tr -d '\\' | xargs apt-get -y install


# Build Gazebo
git clone https://github.com/osrf/gazebo.git /tmp/gazebo \
    && cd /tmp/gazebo \
    && git checkout $GAZEBO_VERSION \
    && mkdir build && cd build  \
    && cmake ../ \
    && make $MAKE_FLAGS \
    && make install 

# /usr/local/lib is not on the load path by defalut so add it
echo '/usr/local/lib' | sudo tee /etc/ld.so.conf.d/gazebo.conf
ldconfig

# Install GymFC dependencies 
apt-get update && apt-get install -y \
    python3-pip \
    python3 
