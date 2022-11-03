#!/bin/bash
DOCKER_TAG=pc-ros-noetic:8.4_tid3
DOCKER_DIR=$HOME/j7ros_home/tid3_ws/src/ti-d3-mobile-robot-demos/docker
if [ "$#" -lt 1 ]; then
    CMD=/bin/bash
else
    CMD="$@"
fi
# modify the server and proxy URLs as requied
ping bitbucket.itg.ti.com -c 1 > /dev/null 2>&1
if [ "$?" -eq "0" ]; then
    USE_PROXY=1
else
    USE_PROXY=0
fi
xhost +local:$$USER
docker run -it --rm \
    -v $HOME/j7ros_home:/root/j7ros_home \
    -v /dev:/dev \
    --privileged \
    --network host \
    --env USE_PROXY=$USE_PROXY \
    --env J7_IP_ADDR=$J7_IP_ADDR \
    --env PC_IP_ADDR=$PC_IP_ADDR \
    --env='DISPLAY' \
    --env='QT_X11_NO_MITSHM=1' \
    --volume='/tmp/.X11-unix:/tmp/.X11-unix:rw' \
      $DOCKER_TAG $CMD
xhost -local:$$USER
