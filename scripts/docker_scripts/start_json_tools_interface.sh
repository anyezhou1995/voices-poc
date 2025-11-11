#!/bin/bash

docker exec -it --user dt_user dt-core bash -c 'source ~/.dt_site_config && cd distributed-testing/scripts/json_scripts && python3 json_tools_interface.py'