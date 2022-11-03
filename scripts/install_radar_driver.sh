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

# install the radar ROS node
CURRENT_DIR=$(pwd)
WORK_PATH=$PROJ_DIR/nodes/radar_driver
mkdir -p $WORK_PATH
cd $WORK_PATH
if [[ ! -d "ti_mmwave_tracker_rospkg" ]]; then
    git clone --single-branch --branch master https://git.ti.com/git/mmwave_radar/mmwave_ti_ros.git
    cd mmwave_ti_ros
    git checkout 6323d2aed92548c963e73d2fb7ca2d3ffb972827
    cd -
    cp -r mmwave_ti_ros/ros_driver/src/ti_mmwave_tracker_rospkg .
    cp -r mmwave_ti_ros/ros_driver/src/serial .
    rm -rf mmwave_ti_ros
fi

cd $CURRENT_DIR
