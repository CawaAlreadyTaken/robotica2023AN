#!/bin/bash

VISIONNODE_COMMAND="python3 ${HOME}/robotica2023AN/src/vision/src/visionNode.py & echo 'Vision node started'"
UR5_COMMAND="python3 ${HOME}/ros_ws/src/locosim/robot_control/lab_exercises/lab_palopoli/ur5_generic.py $1"

echo "Starting $1"
echo "Starting vision node may take a while..."

stty -echoctl # hide ^C

source ${HOME}/robotica2023AN/devel/setup.bash

function stopCommand() {
    echo "Stopping $1"
    pkill -f "ur5_generic.py $1"
    pkill -f "visionNode.py"
    pkill -f "assignment1"
    pkill -f "assignment2"
    pkill -f "assignment3"
    pkill -f "assignment4"
    stty echoctl # show ^C
    exit 0
}

trap 'stopCommand' SIGINT

case "$1" in
    "ass1") $UR5_COMMAND  2>/dev/null & rosrun control assignment1 >/dev/null & $VISIONNODE_COMMAND >/dev/null 2>&1
    ;;
    "ass2") $UR5_COMMAND >/dev/null 2>/dev/null & rosrun control assignment2  & $VISIONNODE_COMMAND >/dev/null 2>&1
    ;;
    "ass3") $UR5_COMMAND >/dev/null 2>/dev/null & rosrun control assigmnent3 >/dev/null & $VISIONNODE_COMMAND
    ;;
    "ass4") $UR5_COMMAND >/dev/null 2>/dev/null & rosrun control assigmnent4 >/dev/null & $VISIONNODE_COMMAND
    ;;
esac
