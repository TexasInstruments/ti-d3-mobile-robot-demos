#!/bin/bash
# project path
ARCH=`arch`
PROJ_NAME=ti-d3-mobile-robot
if [[ "$ARCH" == "aarch64" ]]; then
    PROJ_DIR=/opt/$PROJ_NAME
elif [[ "$ARCH" == "x86_64" ]]; then
    PROJ_DIR=$HOME/j7ros_home/tid3_ws/src/$PROJ_NAME
else
    echo "$ARCH is not supported"
    exit 1
fi

# install D3 ROS nodes
GIT_TAG=08.04.00
GIT_URL_BASE=https://github.com/D3Engineering
CURRENT_DIR=$(pwd)
WORK_PATH=$PROJ_DIR/nodes/d3_nodes
mkdir -p $WORK_PATH

function git_clone_with_tag {
    GIT_URL=$1
    GIT_FOLDER=$2
    TAG=$3
    cd $WORK_PATH
    if [[ ! -d "$GIT_FOLDER" ]]; then
        git clone --single-branch --branch master $GIT_URL $GIT_FOLDER
        cd $GIT_FOLDER
        git checkout tags/$TAG -b $TAG
        cd $WORK_PATH
    else
        echo "$GIT_FOLDER already exists"
    fi
}

git_clone_with_tag $GIT_URL_BASE/edge-ai-ros-fusion.git   d3_fusion   $GIT_TAG
git_clone_with_tag $GIT_URL_BASE/edge-ai-ros-motorctl.git d3_motorctl $GIT_TAG
git_clone_with_tag $GIT_URL_BASE/edge-ai-ros-gamepad.git  d3_gamepad  $GIT_TAG

cd $CURRENT_DIR
