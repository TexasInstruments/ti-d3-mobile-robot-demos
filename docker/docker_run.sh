#!/bin/bash
DOCKER_TAG=j7-ros-noetic:8.4_tid3
DOCKER_DIR=/opt/ti-d3-mobile-robot-demos/docker
IP_ADDR=$(ifconfig | grep -A 1 'eth0' | tail -1 | awk '{print $2}')
if [[ ! $IP_ADDR =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    IP_ADDR=$(ifconfig | grep -A 1 'wlp1s0' | tail -1 | awk '{print $2}')
fi
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
docker run -it --rm \
    -v /home/root/j7ros_home:/root/j7ros_home \
    -v /opt/robotics_sdk:/opt/robotics_sdk \
    -v /opt/ti-d3-mobile-robot-demos:/opt/ti-d3-mobile-robot-demos \
    -v /opt/edge_ai_apps:/opt/edge_ai_apps \
    -v /opt/imaging:/opt/imaging \
    -v /opt/model_zoo:/opt/model_zoo \
    -v /usr/include/dlpack:/usr/include/dlpack \
    -v /home/root/j7ros_home/.ros:/root/.ros \
    -v /dev:/dev \
    --privileged \
    --network host \
    --env USE_PROXY=$USE_PROXY \
    --env TIVA_LIB_VER=8.4.0 \
    --env J7_IP_ADDR=$IP_ADDR \
    --env-file $DOCKER_DIR/env_list.txt \
    --device-cgroup-rule='c 235:* rmw' \
      $DOCKER_TAG $CMD

