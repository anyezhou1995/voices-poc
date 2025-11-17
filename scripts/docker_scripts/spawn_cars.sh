#!/bin/bash

docker exec -it --user root dt-core bash -c 'export HOME=/home/dt_user && export VUG_CARLA_EGG_DIR=$HOME/CARLA/PythonAPI/ \
&& cd $HOME/distributed-testing/scripts/carla_python_scripts/ \
&& python3 spectator_view_delave.py && python3 spawn_npc.py --number-of-vehicles 30'