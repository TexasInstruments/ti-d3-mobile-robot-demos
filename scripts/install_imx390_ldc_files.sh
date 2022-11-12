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

unzip $PROJ_DIR/scripts/tda4_files/imx390_35244_ldc_bin_files.zip \
    -d /opt/imaging/imx390
