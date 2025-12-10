#!/bin/bash

docker exec -it --user root dt-core bash -c 'export HOME=/home/dt_user && export VUG_CARLA_EGG_DIR=$HOME/CARLA/PythonAPI/ \
&& cd $HOME/distributed-testing/scripts/carla_python_scripts/ORNL_EcoDriving/ \
&& python3 manual_control_keyboard_msgLive_thread.py --x -722.989 --y 741.432 --z 0 --rolename ORNL-AUTO-1 --filter vehicle.tesla.model3 --outfile testRecording.csv'