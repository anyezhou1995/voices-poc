#!/bin/bash

docker exec -it dt-core bash -c 'export VUG_CARLA_EGG_DIR=$HOME/CARLA/PythonAPI && $HOME/distributed-testing/scripts/run_scripts/start-debug-carla-adapter.sh > $HOME/tls.txt && cd $HOME/distributed-testing/scripts/carla_python_scripts/ && pwd && python3 $HOME/distributed-testing/scripts/carla_python_scripts/get_trafficSignal_actorID.py -f $HOME/tls.txt && python3 $HOME/distributed-testing/scripts/carla_python_scripts/draw_landmarks.py '
