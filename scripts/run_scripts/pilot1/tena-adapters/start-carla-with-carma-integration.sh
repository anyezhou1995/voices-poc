#!/bin/bash
trap cleanup SIGINT

function cleanup {
	echo "Stopping CARMA Simulation"
	docker kill carla_carma_integration
	pkill -9 CarlaUE4
       kill -9 $set_time_mode_pid
	exit
}

. ../../../../config/node_info.config

if [[ $carmaID == "TFHRC-CAR-1" ]]
then
       SPAWN_PT="255, -230, 1, 0, 0, 0" # latitude=0.002066, longitude=0.002291, altitude=1.000000
elif [[ $carmaID == "TFHRC-CAR-2" ]]
then
       SPAWN_PT="215, -169.4, 1, 0, 0, 0" # latitude=0.001522, longitude=0.001931, altitude=1.000000
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


$localCarlaPath/CarlaUE4.sh > $CARLA_LOG 2>&1 &

carla_pid=$!
echo "CARLA PID: "$carla_pid
sleep 5

python3 $voicesPocPath/scripts/carla_python_scripts/spectator_veiw_town_04.py

python3 $voicesPocPath/scripts/carla_python_scripts/blank_traffic_signals.py

# set time mode producing faster that real time clock, disabled for Pilot 1 tests 1-3
# nohup python3 $voicesPocPath/scripts/carla_python_scripts/set_time_mode.py 2>&1 > $SET_TIME_MODE_LOG & 

# set_time_mode_pid=$!
# echo "Set time mode PID: "$set_time_mode_pid

echo
echo "----- SUCCESSFULLY SET TIME MODE, CONTINUOUSLY TICKING WORLD -----"
echo      


docker run \
	   -it -d --rm \
       --name carla_carma_integration \
       --net=host \
       usdotfhwastol/carma-carla-integration:carma-carla-1.0.0-voices-color \

docker exec \
       -it \
       carla_carma_integration \
       bash -c \
       "export PYTHONPATH=$PYTHONPATH:/home/PythonAPI/carla-0.9.10-py2.7-linux-x86_64.egg &&
       source /home/carla_carma_ws/devel/setup.bash &&
       roslaunch carla_carma_agent carla_carma_agent.launch town:=\"$carlaMapName\" vehicle_color_ind:=\"0\" spawn_point:=\"$SPAWN_PT\"" &>> $SIM_LOG

cleanup
