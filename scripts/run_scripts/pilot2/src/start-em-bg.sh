#!/bin/bash

# Start Execution Manager

voices_site_config=$HOME/.voices_site_config
voices_scenario_config=$HOME/.voices_scenario_config

if [ -L ${voices_site_config} ] && [ -L ${voices_scenario_config} ]; then
    if [ -e ${voices_site_config} ] && [ -e ${voices_scenario_config} ]; then
        site_config_link_dest=$(readlink -f $voices_site_config)
        site_link_base_name=$(basename ${site_config_link_dest})

        scenario_config_link_dest=$(readlink -f $voices_scenario_config)
        scenario_link_base_name=$(basename ${scenario_config_link_dest})

        source $HOME/.voices_site_config

		# if voices config docker exists, then source it to overwrite docker specific vars
		if [ -e ${voices_site_config_docker} ]; then
			source $HOME/.voices_site_config_docker
		fi

		source $HOME/.voices_scenario_config

		if [ -e ${voices_scenario_config_docker} ]; then
			source $HOME/.voices_scenario_config_docker
		fi

        echo "Site Config: "$site_link_base_name
        echo "Scenario Config: "$scenario_link_base_name
    else
        echo "[!!!] .voices_site_config or .voices_scenario_config link is broken"
        echo "Site Config: "$(readlink -f $site_link_base_name)
        echo "Scenario Config: "$(readlink -f $scenario_link_base_name)
        exit 1
   fi
elif [ -e ${voices_site_config} ] || [ -e ${voices_site_config} ]; then
    echo "[!!!] .voices_site_config or .voices_scenario_config file is not a symbolic link"
    echo "Site Config: "$(readlink -f $site_link_base_name)
    echo "Scenario Config: "$(readlink -f $scenario_link_base_name)
    exit 1
else
    echo "[!!!] .voices_site_config or .voices_scenario_config symbolic link does not exist"
    echo "Site Config: "$(readlink -f $site_link_base_name)
    echo "Scenario Config: "$(readlink -f $scenario_link_base_name)
    exit 1
fi

$VUG_EM_PATH/bin/executionManager \
    -listenendpoints $VUG_EM_ADDRESS:$VUG_EM_PORT \
    -logDir $VUG_EM_PATH/log \
    -recoveryDir $VUG_EM_PATH/save \
    -connectionTimeoutInMilliseconds 10000 -twowayTimeoutInMilliseconds 20000 \
    -transientCommunicationAttempts 3 -disconnectTimeoutInMilliseconds 5000 \
    -quiet -nonInteractive &
