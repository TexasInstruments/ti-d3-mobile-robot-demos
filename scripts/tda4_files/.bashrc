#!/bin/bash
cd /opt/edge_ai_apps
source ./init_script.sh

# path
export PATH=$HOME/bin:$PATH
export PROJ_DIR=/opt/ti-d3-mobile-robot-demos

# alias
alias eb="vi ~/.bashrc"
alias sb="source ~/.bashrc"
alias setup_cam="/opt/edge_ai_apps/scripts/setup_cameras.sh"
alias cw="cd $ROS_WS"
alias cs="cd /opt/ti-d3-mobile-robot-demos"
alias cj="cd $HOME/j7ros_home"

cd $PROJ_DIR
