#!/bin/bash 

docker exec --user root dt-core bash -c 'export HOME=/home && export VUG_CARLA_EGG_DIR=$HOME/CARLA/PythonAPI && cd $HOME/distributed-testing/scripts/carla_python_scripts/ && python3 manual_control_keyboard_virtual_pilot2.py --follow_vehicle carma_1'