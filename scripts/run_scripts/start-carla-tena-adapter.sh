#/bin/bash

#  *
#  * Copyright (C) 2022 LEIDOS.
#  *
#  * Licensed under the Apache License, Version 2.0 (the "License"); you may not
#  * use this file except in compliance with the License. You may obtain a copy o\
# f
#  * the License at
#  *
#  * http://www.apache.org/licenses/LICENSE-2.0
#  *
#  * Unless required by applicable law or agreed to in writing, software
#  * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#  * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
#  * License for the specific language governing permissions and limitations under
#  * the License.
#  *

dt_site_config=$HOME/.dt_site_config
dt_scenario_config=$HOME/.dt_scenario_config

dt_site_config_docker=$HOME/.dt_site_config_docker
dt_scenario_config_docker=$HOME/.dt_scenario_config_docker

if [ -L ${dt_site_config} ] && [ -L ${dt_scenario_config} ]; then
    if [ -e ${dt_site_config} ] && [ -e ${dt_scenario_config} ]; then
        site_config_link_dest=$(readlink -f $dt_site_config)
        site_link_base_name=$(basename ${site_config_link_dest})

        scenario_config_link_dest=$(readlink -f $dt_scenario_config)
        scenario_link_base_name=$(basename ${scenario_config_link_dest})

        source $HOME/.dt_site_config

        # if dt config docker exists, then source it to overwrite docker specific vars
        if [ -e ${dt_site_config_docker} ]; then
            source $HOME/.dt_site_config_docker
        fi

        source $HOME/.dt_scenario_config

        if [ -e ${dt_scenario_config_docker} ]; then
            source $HOME/.dt_scenario_config_docker
        fi

        echo "Site Config: "$site_link_base_name
        echo "Scenario Config: "$scenario_link_base_name
    else
        echo "[!!!] .dt_site_config or .dt_scenario_config link is broken"
        echo "Site Config: "$(readlink -f $site_link_base_name)
        echo "Scenario Config: "$(readlink -f $scenario_link_base_name)
        exit 1
   fi
elif [ -e ${dt_site_config} ] || [ -e ${dt_site_config} ]; then
    echo "[!!!] .dt_site_config or .dt_scenario_config file is not a symbolic link"
    echo "Site Config: "$(readlink -f $site_link_base_name)
    echo "Scenario Config: "$(readlink -f $scenario_link_base_name)
    exit 1
else
    echo "[!!!] .dt_site_config or .dt_scenario_config symbolic link does not exist"
    echo "Site Config: "$(readlink -f $site_link_base_name)
    echo "Scenario Config: "$(readlink -f $scenario_link_base_name)
    exit 1
fi

localadapterPath=$VUG_LOCAL_INSTALL_PATH/$VUG_CARLA_ADAPTER_VERSION

adapterVerbosity='1'

useBestEffort=''
if [[ $VUG_USE_BEST_EFFORT == true ]]; then
    useBestEffort='-bestEffort'
fi

siteID=$(( $(printf '%s' "$VUG_SHORT_IDENTIFIER" | cksum | awk '{print $1}') & 0xFFFF ))
applicationID=$(( $(printf '%s' "$VUG_CARLA_ADAPTER_VERSION" | cksum | awk '{print $1}') & 0xFFFF ))

echo "----- STARTING VEHICLE E-BRAKE SCRIPT -----"

python3 $VUG_LOCAL_DT_PATH/scripts/carla_python_scripts/stop_vehicles.py 2>&1 | awk '{ print "STOP VEHICLES: ", $0; fflush(); }'&

mkdir -p $VUG_ADAPTER_LOG_PATH

adapterLogFile=$VUG_ADAPTER_LOG_PATH/carla_adapter_terminal_out.log

echo "<< ***** Adapter Started **** >>" > $adapterLogFile
date >> $adapterLogFile

# open a new file descriptor for logging
exec 4>> $adapterLogFile

# redirect trace logs to fd 4
BASH_XTRACEFD=4

set -x

$localadapterPath/bin/CARLAtenaAdapter $useBestEffort -emEndpoints $VUG_EM_ADDRESS:$VUG_EM_PORT -listenEndpoints $VUG_LOCAL_ADDRESS -carlaHost $VUG_CARLA_ADDRESS -simId $VUG_SIM_ID -verbosity $adapterVerbosity -siteID $siteID -applicationID $applicationID -vehiclePublishRate 10 2>&1 | awk -v adapter="[$VUG_CARLA_ADAPTER_VERSION]" '{ print adapter, $0; fflush(); }' | tee -a $adapterLogFile
