#!/bin/bash

#  *
#  * Copyright (C) 2025 LEIDOS.
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

adapterVerbosity='1'

mkdir -p $VUG_ADAPTER_LOG_PATH

adapterLogFile=$VUG_ADAPTER_LOG_PATH/json_publisher_terminal_out.log

useBestEffort=''
if [[ $VUG_USE_BEST_EFFORT == true ]]; then
    useBestEffort='-bestEffort'
fi

echo "<< ***** Adapter Started ***** >>" > $adapterLogFile
date >> $adapterLogFile

# open a new file descriptor for logging
exec 4>> $adapterLogFile

# redirect trace logs to fd 4
BASH_XTRACEFD=4

set -x

$VUG_JSON_PUBLISHER_PATH/start.sh $useBestEffort -emEndpoints $VUG_EM_ADDRESS:$VUG_EM_PORT -listenEndpoints $VUG_LOCAL_ADDRESS -serverEndpoint $VUG_LOCAL_ADDRESS:8004 -verbosity $adapterVerbosity | awk -v adapter="[$VUG_JSON_PUBLISHER_VERSION]" '{print adapter, $0, fflush(); }' | tee -a $adapterLogFile