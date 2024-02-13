#!/bin/bash
trap cleanup SIGINT

function cleanup {
    echo "----- STOPPING CARMA PLATFORM -----"
	sudo -u carma carma stop all
}

function print_help {
	echo
	echo "Pilot 1 Test 4 usage: --map smart_intersection --no_tick"
	echo
	echo "usage: start-carla.sh [--no_tick] [--low_quality] [--help]"
	echo
	echo "Start CARLA Simulator for VOICES"
	echo
	echo "optional arguments:"
	echo "    --help             show help"
	echo
}

voices_site_config=$HOME/.voices_site_config
voices_scenario_config=$HOME/.voices_scenario_config

if [ -L ${voices_site_config} ] && [ -L ${voices_scenario_config} ]; then
    if [ -e ${voices_site_config} ] && [ -e ${voices_scenario_config} ]; then
        site_config_link_dest=$(readlink -f $voices_site_config)
        site_link_base_name=$(basename ${site_config_link_dest})

        scenario_config_link_dest=$(readlink -f $voices_scenario_config)
        scenario_link_base_name=$(basename ${scenario_config_link_dest})

        source $voices_site_config
        source $voices_scenario_config

        echo "Site Config: "$VUG_SITE_CONFIG_FILE
        echo "Scenario Config: "$VUG_SCENARIO_CONFIG_FILE
    else
        echo "[!!!] .voices_site_config or .voices_scenario_config link is broken"
        echo "Site Config: "$(readlink -f $voices_site_config)
        echo "Scenario Config: "$(readlink -f $voices_scenario_config)
        exit 1
   fi
elif [ -e ${voices_site_config} ] || [ -e ${voices_site_config} ]; then
    echo "[!!!] .voices_site_config or .voices_scenario_config file is not a symbolic link"
    echo "Site Config: "$(readlink -f $voices_site_config)
    echo "Scenario Config: "$(readlink -f $voices_scenario_config)
    exit 1
else
    echo "[!!!] .voices_site_config or .voices_scenario_config symbolic link does not exist"
    echo "Site Config: "$(readlink -f $voices_site_config)
    echo "Scenario Config: "$(readlink -f $voices_scenario_config)
    exit 1
fi

mkdir -p $VUG_CARMA_SIM_LOG_PATH

CARLA_LOG=$VUG_CARMA_SIM_LOG_PATH/voices_carla_simulator.log
SIM_LOG=$VUG_CARMA_SIM_LOG_PATH/voices_carla_carma_integration.log
SET_TIME_MODE_LOG=$VUG_CARMA_SIM_LOG_PATH/set_time_mode.log

echo "" >> $CARLA_LOG
echo "<< ***** New Session **** >>" >> $CARLA_LOG
date >> $CARLA_LOG
echo "" >> $SIM_LOG
echo "<< ***** New Session **** >>" >> $SIM_LOG
date >> $SIM_LOG


no_tick_enabled=false
timeSyncEnabled=false
low_quality_flag=""
next_flag_is_map=false
carla_map=""


for arg in "$@"
do

	if [[ $arg == "--help" ]]; then
		
		print_help
		exit

	elif [[ $arg != "" ]]; then
		
		echo
		echo "Invalid argument: $arg"
		print_help
		exit

	fi
done

echo "----- STARTING VEHICLE E-BRAKE SCRIPT -----"

$VUG_LOCAL_VOICES_POC_PATH/docker/other_scripts/stop_all_vehicles.sh &

echo "----- STARTING CARMA PLATFORM -----"

sudo -u carma carma start all -d

echo "----- WAITING FOR CARMA PLATFORM TO STARTUP -----"

sleep 7

echo "----- STARTING CARLA-CARMA INTEGRATION TOOL -----"

$VUG_LOCAL_VOICES_POC_PATH/scripts/run_scripts/pilot2/src/start-carma-carla-integration.sh

cleanup
