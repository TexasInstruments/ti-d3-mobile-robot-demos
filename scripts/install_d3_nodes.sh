#!/bin/bash
# project path
ARCH=`arch`
PROJ_NAME=ti-d3-mobile-robot-demos
if [[ "$ARCH" == "aarch64" ]]; then
    PROJ_DIR=/opt/$PROJ_NAME
elif [[ "$ARCH" == "x86_64" ]]; then
    PROJ_DIR=$HOME/j7ros_home/tid3_ws/src/$PROJ_NAME
else
    echo "$ARCH is not supported"
    exit 1
fi
CURRENT_DIR=`pwd`

# install D3 ROS nodes
GIT_TAG=08.04.00
GIT_URL_BASE=https://github.com/D3Engineering
CURRENT_DIR=$(pwd)
WORK_PATH=$PROJ_DIR/nodes/d3_nodes
mkdir -p $WORK_PATH

function git_clone_with_tag {
    GIT_URL=$1
    GIT_FOLDER=$2
    GIT_BRANCH=$3
    TAG=$4
    cd $WORK_PATH
    if [[ ! -d "$GIT_FOLDER" ]]; then
        git clone --branch $GIT_BRANCH $GIT_URL $GIT_FOLDER
        cd $GIT_FOLDER
        git checkout tags/$TAG -b $TAG
        cd $WORK_PATH
    else
        echo "$GIT_FOLDER already exists"
    fi
}

function git_clone_commit {
    GIT_URL=$1
    GIT_FOLDER=$2
    GIT_BRANCH=$3
    GIT_COMMIT=$4
    cd $WORK_PATH
    if [[ ! -d "$GIT_FOLDER" ]]; then
        git clone --branch $GIT_BRANCH $GIT_URL $GIT_FOLDER
        cd $GIT_FOLDER
        git checkout $GIT_COMMIT
        cd $WORK_PATH
    else
        echo "$GIT_FOLDER already exists"
    fi
}

git_clone_commit $GIT_URL_BASE/edge-ai-ros-fusion.git   d3_fusion   fusion-support-x4 c4e226b682c7e64550e12619d7b828058d51b193
git_clone_commit $GIT_URL_BASE/edge-ai-ros-motorctl.git d3_motorctl master            be1f7c86f48bd2a48e7e8dacbadeee1ed4c1e8b1
git_clone_commit $GIT_URL_BASE/edge-ai-ros-gamepad.git  d3_gamepad  master            611cc069bbdd2ff11498e3b516bc0519576c9130

#TODO: remove below once the patch is applied directly to d3_fusion repo
cd $WORK_PATH/d3_fusion && git apply $PROJ_DIR/scripts/patches/d3_fusion_4x.patch && cd -

cd $CURRENT_DIR
