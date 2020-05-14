# A container for the GymFC environment 
# Get with docker pull wil3/gymfc:v1
FROM ubuntu:bionic

ENV USER=gymfc
RUN useradd -ms /bin/bash $USER && mkdir /home/$USER/local

# Install Dart dependencies
RUN apt-get update && apt-get -y install \
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
RUN apt-get update && apt-get -y install \
    liboctomap-dev

# Build Dart
ARG DART_VERSION=v6.7.0
RUN git clone "https://github.com/dartsim/dart.git" /tmp/dart \
    && cd /tmp/dart && git checkout $DART_VERSION \
    && mkdir build && cd build \
    && cmake .. \
    && make -j4 \
    && make install \
    && rm -rf /tmp/dart

# Prepare Gazebo
RUN apt-get update && apt-get -y install \
    lsb-release \
    wget

RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
    && wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - && apt-get update \
    && wget https://bitbucket.org/osrf/release-tools/raw/default/jenkins-scripts/lib/dependencies_archive.sh -O /tmp/dependencies.sh

# Install Gazebo dependencies
RUN /bin/bash -c "GAZEBO_MAJOR_VERSION=10 ROS_DISTRO=dummy source /tmp/dependencies.sh && echo \${BASE_DEPENDENCIES} \${GAZEBO_BASE_DEPENDENCIES} \${DART_DEPENDENCIES} | tr -d '\\' | xargs apt-get -y install"


# Build Gazebo
ARG GAZEBO_VERSION=gazebo10_10.1.0
RUN apt-get install -y mercurial libboost-all-dev \
    && hg clone https://bitbucket.org/osrf/gazebo /tmp/gazebo \
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
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3 

USER $USER
WORKDIR /home/$USER

ENTRYPOINT ["/bin/bash", "-c"]
