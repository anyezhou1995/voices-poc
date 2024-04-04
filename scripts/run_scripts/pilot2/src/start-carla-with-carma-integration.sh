#!/bin/bash
trap cleanup SIGINT

function cleanup {
	echo "Stopping CARMA Simulation"
	sleep 3s
	docker kill carma_carla_integration
	pkill -9 CarlaUE4
  if [[ $timeSyncEnabled == "true" ]]; then
    kill -9 $set_time_mode_pid
  fi
	exit
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
	echo "    --no_tick          do not set time mode and tick simulation"
	echo "    --low_quality      start CARLA in low quality mode"
	echo "    --map <map name>   start carla with given map name"
	echo "                           Currently supported maps:"
	echo "                             - Town04"
	echo "                             - smart_intersection"
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

if [[ -f $VUG_LOCAL_CARLA_PATH/CarlaUE4.sh ]]; then
	echo "Found CARLA Simulator"
else
	echo
	echo "CARLA Simulator not found at $VUG_LOCAL_CARLA_PATH/CarlaUE4.sh"
	exit
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
	if [ "$next_flag_is_map" = true ]; then

		carla_map=$arg
		next_flag_is_map=false
	
	elif [[ $arg == "--no_tick" ]]; then
		
		no_tick_enabled=true

	elif [[ $arg == "--low_quality" ]]; then
		
		low_quality_flag="-quality-level=Low"

	elif [[ $arg == "--map" ]]; then
		
		next_flag_is_map=true

	elif [[ $arg == "--help" ]]; then
		
		print_help
		exit

	elif [[ $arg != "" ]]; then
		
		echo
		echo "Invalid argument: $arg"
		print_help
		exit

	fi
done

# require map becuase it determines spawn points
if [[ $carla_map == "" ]]; then

		echo
		echo "[!!!] --map flag is required"
		print_help
		exit
	
fi

$VUG_LOCAL_CARLA_PATH/CarlaUE4.sh $low_quality_flag > $CARLA_LOG 2>&1 &

carla_pid=$!
echo "CARLA PID: "$carla_pid
sleep 7s


if [[ $carla_map == "Town04" ]]; then

	echo "Changing map to: $carla_map"
	python3 $VUG_LOCAL_VOICES_POC_PATH/scripts/carla_python_scripts/config.py -m $carla_map
	sleep 5s

	echo "Changing perspective to Test Intersection"

	python3 $VUG_LOCAL_VOICES_POC_PATH/scripts/carla_python_scripts/spectator_view_town_04.py

	if [[ $VUG_CARMA_VEHICLE_ID == "TFHRC-CAR-1" ]]
	then
		SPAWN_PT="255,-230,1,0,0,0" # latitude=0.002066, longitude=0.002291, altitude=1.000000
	elif [[ $VUG_CARMA_VEHICLE_ID == "TFHRC_CAR_2" ]]
	then
		SPAWN_PT="215,-169.4,1,0,0,0" # latitude=0.001522, longitude=0.001931, altitude=1.000000
	fi

elif [[ $carla_map == "smart_intersection" ]]; then

	echo "Changing map to: $carla_map"
	python3 $VUG_LOCAL_VOICES_POC_PATH/scripts/carla_python_scripts/config.py -m $carla_map --weather ClearNoon
	sleep 5s

  echo "Changing perspective to simulation site: $VUG_SIM_ID"

    if [[ $VUG_SIM_ID == "CARLA-TFHRC-1" ]]; then
    	python3 $VUG_LOCAL_VOICES_POC_PATH/scripts/carla_python_scripts/spectator_view_smart_intersection.py 59.992634 195.027710 17.727715 -29.558195 -125.864532 0.002484
    elif [[ $VUG_SIM_ID == "CARLA_TFHRC_2" ]]; then
    	python3 $VUG_LOCAL_VOICES_POC_PATH/scripts/carla_python_scripts/spectator_view_smart_intersection.py 28.327816 139.781906 16.607105 -25.901394 56.039539 0.000042
    fi

	if [[ $VUG_CARMA_VEHICLE_ID == "TFHRC_CAR_2" ]]
	then
		SPAWN_PT="34,250,0,0,0,85" # latitude=34.067713, longitude=-118.445144, altitude=1.000000
#		SPAWN_PT="31.2,308.3,0,0,0,85" # latitude=34.067713, longitude=-118.445144, altitude=1.000000
	elif [[ $VUG_CARMA_VEHICLE_ID == "UCLA-OPENCDA" ]]
	then
		SPAWN_PT="30.0,340.5,1,0,0,85" # latitude=34.068104, longitude=-118.445083, altitude=1.000000 #
	fi

else
	echo
	echo "Map not supported: $carla_map"
	echo
	cleanup
	
fi

python3 $VUG_LOCAL_VOICES_POC_PATH/scripts/carla_python_scripts/blank_traffic_signals.py


if [ "$no_tick_enabled" = true ]; then

	echo "Not ticking clock"

else

  # set time mode producing faster that real time clock, disabled for Pilot 1 tests 1-3
  echo "Setting time mode."
  nohup python3 $VUG_LOCAL_VOICES_POC_PATH/scripts/carla_python_scripts/set_time_mode.py 2>&1 > $SET_TIME_MODE_LOG &
  set_time_mode_pid=$!
  echo "Set time mode PID: "$set_time_mode_pid
  echo
  echo "----- SUCCESSFULLY SET TIME MODE, CONTINUOUSLY TICKING WORLD -----"
  echo
fi


echo "----- STARTING CARLA-CARMA INTEGRATION TOOL -----"

$VUG_LOCAL_VOICES_POC_PATH/scripts/run_scripts/pilot2/src/start-carma-carla-integration.sh

cleanup
