# For debugging 
source /usr/share/gazebo-8/setup.sh
export GAZEBO_MODEL_PATH=${PWD}/models:${GAZEBO_MODEL_PATH}
export GAZEBO_RESOURCE_PATH=${PWD}/worlds:${GAZEBO_RESOURCE_PATH}
export GAZEBO_PLUGIN_PATH=${PWD}/plugins/build:${GAZEBO_PLUGIN_PATH}
gazebo --verbose ${PWD}/worlds/attitude-iris.world
