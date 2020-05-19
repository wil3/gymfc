# Install the dependencies for GymFC on Ubuntu 18.04. 
# Requires sudo


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
DART_VERSION=v6.7.0
git clone "https://github.com/dartsim/dart.git" /tmp/dart \
    && cd /tmp/dart && git checkout $DART_VERSION \
    && mkdir build && cd build \
    && cmake .. \
    && make -j4 \
    && make install \
    && rm -rf /tmp/dart

# Install Gazebo from source following the instructions from here: http://gazebosim.org/tutorials?tut=install_from_source&cat=install

# Prepare Gazebo
apt-get remove '.*gazebo.*' '.*sdformat.*' '.*ignition-math.*' '.*ignition-msgs.*' '.*ignition-transport.*'

apt-get update && apt-get -y install \
    lsb-release \
    wget \
    mercurial \
    libboost-all-dev

sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
    && wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - && apt-get update 


# Install Gazebo dependencies
wget https://raw.githubusercontent.com/ignition-tooling/release-tools/master/jenkins-scripts/lib/dependencies_archive.sh -O /tmp/dependencies.sh
GAZEBO_MAJOR_VERSION=10 
ROS_DISTRO=dummy 
source /tmp/dependencies.sh && echo ${BASE_DEPENDENCIES} ${GAZEBO_BASE_DEPENDENCIES} ${DART_DEPENDENCIES} | tr -d '\\' | xargs apt-get -y install


# Build Gazebo
GAZEBO_VERSION=gazebo10_10.1.0
hg clone https://bitbucket.org/osrf/gazebo /tmp/gazebo \
    && cd /tmp/gazebo \
    && hg up $GAZEBO_VERSION \
    && mkdir build && cd build  \
    && cmake ../ \
    && make -j4 \
    && make install \
    && rm -rf /tmp/gazebo

# Change default location of gazebo install.
# Currently not used as it is most benificial so you can run multiple gazebo versions.
# However to run gymfc the gymfc.ini must point to the correct setup.sh file.
# cmake -DCMAKE_INSTALL_PREFIX=/home/$USER/local ../ \

# Install GymFC dependencies 
apt-get update && apt-get install -y \
    python3-pip \
    python3 
