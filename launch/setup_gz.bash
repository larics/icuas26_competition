#!/bin/bash

# Setup environment to make Crazysim visible to Gazebo.
# Derived from PX4 setup_gazebo.sh file

if [ "$#" != 3 ]; then
    echo -e "usage: source setup_gz.bash src_dir build_dir\n"
    return 1
fi

SRC_DIR=$1
BUILD_DIR=$2
WORLD_DIR=$3 

# setup gz environment and update package path
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:${BUILD_DIR}/build_crazysim_gz
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:${WORLD_DIR}/models:${SRC_DIR}/tools/crazyflie-simulation/simulator_files/gazebo/models:${WORLD_DIR}/worlds
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${BUILD_DIR}/build_crazysim_gz