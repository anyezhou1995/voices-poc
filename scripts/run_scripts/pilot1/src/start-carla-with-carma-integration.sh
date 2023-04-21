#!/bin/bash
trap cleanup SIGINT

function cleanup {
	echo "Stopping CARMA Simulation"
	docker kill carla_carma_integration
	pkill -9 CarlaUE4
       kill -9 $set_time_mode_pid
	exit
}

function print_help {
	echo
	echo "usage: start-carla.sh [--no_tick] [--low_quality] [--help]"
	echo
	echo "Start CARLA Simulator for VOICES"
	echo
	echo "optional arguments:"
	echo "    --no-tick          do not set time mode and tick simulation"
	echo "    --low_quality      start CARLA in low quality mode"
	echo "    --map <map name>   start carla with given map name"
	echo "                           Currently supported maps:"
	echo "                             - Town04"
	echo "                             - smart_intersection"
	echo "    --help             show help"
	echo
}
. ../../../../config/node_info.config

if [[ -f $localCarlaPath/CarlaUE4.sh ]]; then
	echo "Found CARLA Simulator"
else
	echo
	echo "CARLA Simulator not found at $localCarlaPath/CarlaUE4.sh"
	exit
fi

mkdir -p $localCarmaSimLogPath

CARLA_LOG=$localCarmaSimLogPath/voices_carla_simulator.log
SIM_LOG=$localCarmaSimLogPath/voices_carla_carma_integration.log
SET_TIME_MODE_LOG=$localCarmaSimLogPath/set_time_mode.log

echo "" >> $CARLA_LOG
echo "<< ***** New Session **** >>" >> $CARLA_LOG
date >> $CARLA_LOG
echo "" >> $SIM_LOG
echo "<< ***** New Session **** >>" >> $SIM_LOG
date >> $SIM_LOG


no_tick_enabled=false
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

$localCarlaPath/CarlaUE4.sh $low_quality_flag > $CARLA_LOG 2>&1 &

carla_pid=$!
echo "CARLA PID: "$carla_pid
sleep 7s

if [[ $carla_map == "Town04" ]]; then

	echo "Changing map to: $carla_map"
	python3 $voicesPocPath/scripts/carla_python_scripts/config.py -m $carla_map
	sleep 5s

	python3 $voicesPocPath/scripts/carla_python_scripts/spectator_view_town_04.py

	if [[ $carmaID == "TFHRC-CAR-1" ]]
	then
		SPAWN_PT="255, -230, 1, 0, 0, 0" # latitude=0.002066, longitude=0.002291, altitude=1.000000
	elif [[ $carmaID == "TFHRC-CAR-2" ]]
	then
		SPAWN_PT="215, -169.4, 1, 0, 0, 0" # latitude=0.001522, longitude=0.001931, altitude=1.000000
	fi

elif [[ $carla_map == "smart_intersection" ]]; then

	echo "Changing map to: $carla_map"
	python3 $voicesPocPath/scripts/carla_python_scripts/config.py -m $carla_map
	sleep 5s

	python3 $voicesPocPath/scripts/carla_python_scripts/spectator_view_smart_intersection.py

	if [[ $carmaID == "TFHRC-CAR-2" ]]
	then
		SPAWN_PT="44.369656, 86.320465, 1, 0, 0, 263" # latitude=34.067713, longitude=-118.445144, altitude=1.000000
	elif [[ $carmaID == "UCLA-OPENCDA" ]]
	then
		SPAWN_PT="50.003670, 43.160156, 1, 0, 0, 263" # latitude=34.068104, longitude=-118.445083, altitude=1.000000 # 
	fi

elif [[ $carla_map == "" ]]; then

	echo "Loading default CARLA map"

else
	echo
	echo "Map not supported: $carla_map"
	echo
	cleanup
	
fi

python3 $voicesPocPath/scripts/carla_python_scripts/blank_traffic_signals.py


# set time mode producing faster that real time clock, disabled for Pilot 1 tests 1-3
nohup python3 $voicesPocPath/scripts/carla_python_scripts/set_time_mode.py 2>&1 > $SET_TIME_MODE_LOG & 

# set_time_mode_pid=$!
echo "Set time mode PID: "$set_time_mode_pid

echo
echo "----- SUCCESSFULLY SET TIME MODE, CONTINUOUSLY TICKING WORLD -----"
echo      


docker run \
	   -it -d --rm \
       --name carma_carla_integration \
       --net=host \
       usdotfhwastol/carma-carla-integration:K900-test
echo "------------------------exec---------------------------------"
docker exec \
       -it \
       carma_carla_integration \
       bash -c \
       "export PYTHONPATH=$PYTHONPATH:/home/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg && \
    	source /home/carma_carla_ws/devel/setup.bash && \
       roslaunch carma_carla_agent carma_carla_agent.launch spawn_point:='44.369656,86.320465,1,0,0,263' town:='Town04' selected_route:='Voices_Pilot1_Test4_TFHRC-CAR-2' synchronous_mode:='false' speed_Kp:='0.4' speed_Ki:='0.03' speed_Kd:='0'"
	    # &> $SIM_LOG

cleanup
