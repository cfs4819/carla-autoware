#!/bin/bash

export CARLA_AUTOWARE_CONTENTS=/home/autoware/autoware-contents
export PYTHON2_EGG=$(ls /home/autoware/PythonAPI | grep py2.)
export PYTHONPATH=$PYTHONPATH:/home/autoware/PythonAPI/$PYTHON2_EGG

source /home/autoware/carla_ws/devel/setup.bash
source /home/autoware/Autoware/install/setup.bash

echo arg1 town: $1, arg2 spawn_point: $2, arg3 port: $3
roslaunch carla_autoware_agent carla_autoware_agent.launch town:=$1 spawn_point:=$2 port:=$3
tail -f /dev/null
