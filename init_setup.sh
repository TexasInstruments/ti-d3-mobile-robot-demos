#!/bin/bash
# Usage: source init_setup.sh

# Release tag info of the current release
GIT_TAG="08.04.00"
if [ "$#" -eq 1 ]; then
    GIT_TAG=$1
fi
echo "GIT_TAG = $GIT_TAG"

# Git repository
GIT_REPO="https://github.com/TexasInstruments/ti-d3-mobile-robot-demos.git"
BRANCH=master

# Define env variables
export WORK_DIR=$HOME/j7ros_home
export ROS_WS=$WORK_DIR/tid3_ws
export PROJ_NAME=ti-d3-mobile-robot-demos

# Installation path
ARCH=`arch`
if [[ "$ARCH" == "aarch64" ]]; then
    export PROJ_DIR=/opt/$PROJ_NAME
elif [[ "$ARCH" == "x86_64" ]]; then
    export PROJ_DIR=$ROS_WS/src/$PROJ_NAME
else
    echo "$ARCH is not supported"
    exit 1
fi

function git_clone_with_tag {
    GIT_URL=$1
    GIT_FOLDER=$2
    TAG=$3
    cd $PROJ_DIR/..
    if [[ ! -d "$GIT_FOLDER" ]]; then
        git clone --single-branch --branch master $GIT_URL $GIT_FOLDER
        cd $GIT_FOLDER
        git checkout tags/$TAG -b $TAG
    else
        echo "$GIT_FOLDER already exists"
    fi
}

# Git clone the project git repository
cd $PROJ_DIR/..
git_clone_with_tag $GIT_REPO $PROJ_NAME $GIT_TAG

# Install mmWave radar driver node
bash $PROJ_DIR/scripts/install_radar_driver.sh

# Install D3 nodes
bash $PROJ_DIR/scripts/install_d3_nodes.sh

# Apply a patch to use `develp` branch for edgeai-tiovx-modules and edgeai-gst-plugins
if [[ "$ARCH" == "aarch64" ]]; then
    cd /opt/edge_ai_apps
    git apply $PROJ_DIR/scripts/patches/edge_ai_apps_gst_repo.patch
    cd -
fi

# Update dtbo ovelay
# NOTE: This was verified only for one version of Fusion board.
# May not be compatible with other version of the Fusion board.
if [[ "$ARCH" == "aarch64" ]]; then
    cp $PROJ_DIR/scripts/tda4_files/uenv.txt /run/media/mmcblk0p1/uenv.txt
    echo "/run/media/mmcblk0p1/uenv.txt updated. Please reboot the TDA4..."
fi

# Install IMX390 LDC bin files
if [[ "$ARCH" == "aarch64" ]]; then
    bash $PROJ_DIR/scripts/install_imx390_ldc_files.sh
fi

# Setup $WORK_DIR
if [[ "$ARCH" == "aarch64" ]]; then
    cp $PROJ_DIR/scripts/tda4_files/{.profile,.bashrc} $HOME
fi
mkdir -p $ROS_WS
cd $PROJ_DIR
