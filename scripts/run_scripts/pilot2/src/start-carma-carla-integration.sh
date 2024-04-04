#!/bin/bash
trap cleanup SIGINT

function cleanup {
	echo "Stopping CARMA CARLA Integration"
	sleep 3s
	docker kill carma_carla_integration
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
# selected_plugins:=\"['/guidance/plugins/route_following_plugin','/guidance/plugins/inlanecruising_plugin','/guidance/plugins/stop_and_wait_plugin','/guidance/plugins/pure_pursuit_wrapper']\" \

#UCLA
#SPAWN_PT="54.25676727294922, 38.415374755859375, 240, 0, 0, 90"

#short test
# SPAWN_PT="56.618145, 45.935390, 240, 0, 0, 90"
# SPAWN_PT="54.25676727294922, 38.415374755859375, 235, 0, 0, 90"

# straightaway
# SPAWN_PT="146.433853, -95.265038, 240.5, 0, 0, 270"
#            role_name:=\"$VUG_CARMA_VEHICLE_ID\"" \

# loop
# SPAWN_PT="109.881569, -65.993828, 240, 0, 0, 270"

# yield test
# SPAWN_POINT="110.145599, -69.313705, 240, 0, 0, 270" 
# Main
#             spawn_point:=\"$VUG_CARMA_SPAWN_POINT\" \

# right before roundabout
# export VUG_CARMA_SPAWN_POINT="93.649223, 69.763718, 236.5, 0, 0, 232.862701"

echo "----- STARTING CARLA-CARMA INTEGRATION TOOL -----"

docker run \
	   -it -d --rm \
       --name carma_carla_integration \
       --net=host \
       usdotfhwastoldev/carma-carla-integration:vug-fds-fix-logs
echo "------------------------exec---------------------------------"
docker exec \
        -it \
        carma_carla_integration \
        bash -c \
        "export ROS_IP=127.0.0.1 && \
        export ROS_MASTER_URI=http://localhost:11311 && \
        export PYTHONPATH=$PYTHONPATH:~/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg:/home/carma/carla-sensor-lib/src && \
        export PYTHONUNBUFFERED=1 && \
        source ~/carma_carla_ws/devel/setup.bash && \
        roslaunch carma_carla_agent carma_carla_agent.launch \
            spawn_point:=\"$VUG_CARMA_SPAWN_POINT\" \
            town:=\"$carla_map\" \
            selected_route:=\"$VUG_CARMA_ROUTE\" \
            synchronous_mode:='true' \
            fixed_delta_seconds:='0.08' \
            use_sim_time:='true' \
            speed_Kp:=0.4 \
            speed_Ki:=0.03 \
            speed_Kd:=0 \
            start_delay_in_seconds:='999' \
            role_name:='carma_1'" \
    &> $SIM_LOG



cleanup
