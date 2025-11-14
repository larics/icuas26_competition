#!/bin/bash

export NUM_ROBOTS=5
export KEYWORD=icuas26_1 #icuas26_1 or empty
export ENV_NAME=${KEYWORD}_world
export SPAWN_POSE_DOC=positions_${KEYWORD}.txt
export GZ_VERSION=garden
export BINVOX_STL_LOCATION=""
export COMM_RANGE=70
export AGV_VEL=0.5
export CHARGING_FILE=charging_$KEYWORD.yaml
export AGV_PATH=AGV_path_$KEYWORD.yaml