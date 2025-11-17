#!/bin/bash

docker exec -it --user root dt-core bash -c 'export HOME=/home/dt_user && export VUG_CARLA_EGG_DIR=$HOME/CARLA/PythonAPI/ \
&& cd $HOME/distributed-testing/scripts/carla_python_scripts/ \
&& (python3 -c "import cv2" >/dev/null 2>&1 || (pip3 uninstall -y opencv-python-headless >/dev/null 2>&1 || true) && pip3 install --user opencv-python) \
&& python3 manual_control_keyboard_bev.py --filter vehicle.*'
