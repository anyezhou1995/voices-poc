#!/bin/bash

docker exec -it --user root voices bash -c 'export HOME=/home && /home/voices-poc/scripts/run_scripts/pilot2/src/start-log-collection.sh'
