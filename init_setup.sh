#!/bin/bash

# Define env variables
ROS_WS=$HOME/j7ros_home/tid3_ws
PROJ_NAME=ti-d3-mobile-robot-demos

# Installation path
ARCH=`arch`
if [[ "$ARCH" == "aarch64" ]]; then
    PROJ_BASE=/opt
elif [[ "$ARCH" == "x86_64" ]]; then
    PROJ_BASE=$ROS_WS/src
else
    echo "$ARCH is not supported"
    exit 1
fi
PROJ_DIR=$PROJ_BASE/$PROJ_NAME

function git_clone_with_tag {
    GIT_URL=$1
    GIT_FOLDER=$2
    TAG=$3
    cd $PROJ_BASE
    if [[ ! -d "$GIT_FOLDER" ]]; then
        git clone --single-branch --branch master $GIT_URL $GIT_FOLDER
        cd $GIT_FOLDER
        git checkout tags/$TAG -b $TAG
    else
        echo "$GIT_FOLDER already exists"
    fi
}

function git_clone {
    GIT_URL=$1
    GIT_FOLDER=$2
    cd $PROJ_BASE
    if [[ ! -d "$GIT_FOLDER" ]]; then
        git clone --single-branch --branch master $GIT_URL $GIT_FOLDER
    else
        echo "$GIT_FOLDER already exists"
    fi
}

# Install mmWave radar driver node
bash $PROJ_DIR/scripts/install_radar_driver.sh

# Install D3 nodes
bash $PROJ_DIR/scripts/install_d3_nodes.sh

# Apply patches to edge_ai_apps:
# to change `develp` branch for edgeai-tiovx-modules and edgeai-gst-plugins
# to establish sybolic links for IMX390 cameras based on the port to which the camera is attached
if [[ "$ARCH" == "aarch64" ]]; then
    cd /opt/edge_ai_apps
    git apply $PROJ_DIR/scripts/patches/edge_ai_apps_gst_repo.patch
    git apply $PROJ_DIR/scripts/patches/edge_ai_apps_setup_cameras.sh.patch
    cd -
fi

# Apply patche to robotics_sdk
# vizThreshold = 0.3
if [[ "$ARCH" == "aarch64" ]]; then
    cd /opt/robotics_sdk
    git apply $PROJ_DIR/scripts/patches/robotics_sdk.patch
    cd -
fi

# Update dtbo ovelay
# NOTE: This was verified only for one version of Fusion board.
# May not be compatible with other version of the Fusion board.
if [[ "$ARCH" == "aarch64" ]]; then
    cp $PROJ_DIR/scripts/tda4_files/uenv.txt /run/media/mmcblk0p1/uenv.txt
    echo "/run/media/mmcblk0p1/uenv.txt updated. Please reboot the TDA4..."
fi

# Establish sybolic links for radar sensor based on USB port to which the sensor is attached
if [[ "$ARCH" == "aarch64" ]]; then
    cp $PROJ_DIR/scripts/tda4_files/10-radar.rules /etc/udev/rules.d
    cp $PROJ_DIR/scripts/tda4_files/radar_load.sh /usr/local/bin
fi

# Install IMX390 LDC bin files
if [[ "$ARCH" == "aarch64" ]]; then
    bash $PROJ_DIR/scripts/install_imx390_ldc_files.sh
fi

# Setup working dir
if [[ "$ARCH" == "aarch64" ]]; then
    cp $PROJ_DIR/scripts/tda4_files/{.profile,.bashrc} $HOME
fi
mkdir -p $ROS_WS
cd $PROJ_DIR
