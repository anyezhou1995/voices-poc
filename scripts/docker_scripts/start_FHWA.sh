#!/bin/bash

docker exec -it --user root dt-core bash -c 'export HOME=/home/dt_user && export VUG_CARLA_EGG_DIR=$HOME/CARLA/PythonAPI/ \
&& cd $HOME/distributed-testing/scripts/carla_python_scripts/ \
&& python3 drive_route.py --x -775 --y 715 --z 0.5 --rolename FHWA-M-3 --dest 607.34 833.78 0.5'