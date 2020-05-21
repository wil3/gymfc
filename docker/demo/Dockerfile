FROM ubuntu:bionic

# install python and dependencies for mesa
RUN apt-get update \
    && apt-get install -y python3-pip python-pip python3-dev \
    && cd /usr/local/bin \
    && ln -s /usr/bin/python3 python \
    && pip3 install --upgrade pip \
    && apt-get install -y xvfb llvm-5.0-runtime x11-utils llvm-dev libxcb-dri2-0-dev libxcb-xfixes0-dev \
    && apt-get install -y libx11-xcb-dev zlib1g-dev xorg-dev xserver-xorg-dev

# compile mesa to be able to use DRI with docker
# enable graphic rendering in a docker container and allow it to forward it to a XHost outside the container.
# The pre-installed mesa drives do not support DRI but Gazebo needs them. Hence we need to install new drivers.
# This takes a while, make that the ADD/COPY commands come later so this layer can be cached by docker build.
RUN mkdir -p /var/tmp/build \
    && cd /var/tmp/build \
    && apt-get install -y wget \
    && wget "https://mesa.freedesktop.org/archive/mesa-18.0.1.tar.gz" \
    && tar xfv mesa-18.0.1.tar.gz \
    && rm mesa-18.0.1.tar.gz \
    && cd mesa-18.0.1 \
    && ./configure --enable-glx=gallium-xlib --with-gallium-drivers=swrast,swr --disable-dri --disable-gbm --disable-egl --enable-gallium-osmesa --prefix=/usr \
    && make -j4 \
    && make install \
    && cd .. \
    && rm -rf mesa-18.0.1

ENV USER=gymfc
RUN useradd -ms /bin/bash $USER && mkdir /home/$USER/local

# prepare Gazebo and Dart build
RUN apt-get install -y lsb-release gnupg2 software-properties-common \
    && apt-get remove '.*gazebo.*' '.*sdformat.*' '.*ignition-math.*' '.*ignition-msgs.*' '.*ignition-transport.*' \
    && sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
    && wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - && apt-get update \
    && wget https://bitbucket.org/osrf/release-tools/raw/default/jenkins-scripts/lib/dependencies_archive.sh -O /tmp/dependencies.sh \
    && apt-get update

# install Gazebo dependencies
RUN /bin/bash -c "GAZEBO_MAJOR_VERSION=10 ROS_DISTRO=dummy source /tmp/dependencies.sh && echo \${BASE_DEPENDENCIES} \${GAZEBO_BASE_DEPENDENCIES} \${DART_DEPENDENCIES} | tr -d '\\' | xargs apt-get -y install"

# Build Dart
ARG DART_VERSION=v6.7.0
RUN apt-get install -y ssh gdb vim git \
    && git clone "https://github.com/dartsim/dart.git" /tmp/dart \
    && cd /tmp/dart && git checkout $DART_VERSION \
    && mkdir build && cd build \
    && cmake .. \
    && make -j4 \
    && make install \
    && rm -rf /tmp/dart

ARG GAZEBO_VERSION=gazebo10_10.1.0
# Build Gazebo
RUN apt-get install -y mercurial libboost-all-dev \
    && hg clone https://bitbucket.org/osrf/gazebo /tmp/gazebo \
    && cd /tmp/gazebo \
    && hg up $GAZEBO_VERSION \
    && mkdir build && cd build  \
    && cmake -DCMAKE_INSTALL_PREFIX=/home/$USER/local ../ \
    && make -j4 \
    && make install \
    && rm -rf /tmp/gazebo

# Finalize mesa installation, remove default ubuntu mesa libs
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y tzdata \
    && ln -fs /usr/share/zoneinfo/Europe/Dublin /etc/localtime \
    && dpkg-reconfigure -f noninteractive tzdata \
    && apt-get install -y mesa-utils libopenmpi-dev python3-tk \
    && dpkg --force-depends -r libgl1 libglx0 va-driver-all libglx-mesa0 libgl1-mesa-dri

ENV XVFB_WHD="1920x1080x24"\
    DISPLAY=":0" \
    LIBGL_ALWAYS_SOFTWARE="1" \
    GALLIUM_DRIVER="llvmpipe" \
    LP_NO_RAST="false" \
    PKG_CONFIG_PATH="/usr/local/pkgconfig" \
    PATH="/home/$USER/local/bin:${PATH}" \
    LD_LIBRARY_PATH="/usr/local/lib:/home/$USER/local/lib:/usr/local/lib/python3.6/dist-packages/gymfc/envs/assets/gazebo/plugins/build/"

# Build gymfc
ADD setup.py gymfc.ini gen_pb.sh LICENSE.txt /gymfc/
ADD gymfc /gymfc/gymfc
RUN pip3 install /gymfc

ADD tests /gymfc/tests
ADD images /gymfc/images

# Download gymfc plugins, build plugins
RUN git clone https://github.com/wil3/gymfc-aircraft-plugins.git /gymfc/demo/plugins \
    && mkdir /gymfc/demo/plugins/build \
    && cd /gymfc/demo/plugins/build \
    && cmake .. \
    && make -j4 \
    && mkdir -p /gymfc/demo/models/solo/plugins/build \
    && cp -r /gymfc/demo/plugins/build/*.so /gymfc/demo/models/solo/plugins/build \
    && rm -rf /gymfc/demo/plugins

RUN chown $USER:$USER -R /gymfc

USER $USER
WORKDIR /home/$USER

ENTRYPOINT ["/bin/bash", "-c"]