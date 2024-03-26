#!/bin/bash

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


echo
read -p "Are you hosting a CARMA Platform Vehicle? [y/n] " isCarmaVehicle

username=$(whoami)

timestamp=$(date -d "today" +"%Y%m%d%H%M%S")

logs_folder_name=$VUG_SIM_ID'_'$timestamp

if [[ $VUG_DOCKER_START_TJ2735_ADAPTER == true ]]; then

    j2735_tcpdump_out="tcpdump -i lo port $VUG_J2735_ADAPTER_RECEIVE_PORT -w J2735_packet_in.pcap"
    j2735_tcpdump_in="tcpdump -i lo port $VUG_J2735_ADAPTER_SEND_PORT -w J2735_packet_out.pcap"

    if [[ $username != "root" ]]; then
        j2735_tcpdump_out="sudo "+$j2735_tcpdump_out
        j2735_tcpdump_in="sudo "+$j2735_tcpdump_in
    fi

fi

if [[ $VUG_DOCKER_START_TJ3224_ADAPTER == true ]]; then

    j3224_tcpdump_out="tcpdump -i lo port $VUG_J3224_ADAPTER_RECEIVE_PORT -w J3224_packet_in.pcap"
    j3224_tcpdump_in="tcpdump -i lo port $VUG_J3224_ADAPTER_SEND_PORT -w J3224_packet_out.pcap"

    if [[ $username != "root" ]]; then
        j3224_tcpdump_out="sudo "+$j3224_tcpdump_out
        j3224_tcpdump_in="sudo "+$j3224_tcpdump_in
    fi

fi

echo
echo "Folder Name: "$logs_folder_name

mkdir -p $VUG_LOG_FILES_ROOT/$logs_folder_name
cd $VUG_LOG_FILES_ROOT/$logs_folder_name

echo
read -p "Would you like to collect the TENA SDO data? [y/n] " save_tdcs_data

if [[ $save_tdcs_data =~ ^[yY]$ ]]; then
    echo
    echo "Starting TDCS" 
    tdcs_command="$VUG_TDCS_PATH/start.sh -emEndpoints $VUG_EM_ADDRESS:$VUG_EM_PORT -listenEndpoints $VUG_LOCAL_ADDRESS -databaseName $logs_folder_name.sqlite -dbFolder ."
    echo $tdcs_command
    $tdcs_command &
fi

echo
read -p "Would you like to collect eco data [y/n] " collect_eco

if [[ $collect_eco =~ ^[yY]$ ]]; then
    echo collecting
    collect_eco_cmd="python3 $HOME/voices-poc/scripts/carla_python_scripts/collect_pilot2_vehicle_eco_data.py --vehicle_rolenames $VUG_COLLECT_ECO_DATA_ROLENAMES --host $VUG_CARLA_ADDRESS --output_dir $VUG_LOG_FILES_ROOT/$logs_folder_name"
fi

#if we are not a live vehicle then prompt to collect logs 
#(live vehicle is not connected to VOICES network)


if [[ $isCarmaVehicle == ^[yY]$ ]]; then
    trap copyCarmaLogs SIGINT
fi

copyCarmaLogs()
{
    echo
    echo "tcpdumps stopped"
    echo
    echo "ENSURE CARMA PLATFORM, CARLA, TDCS, AND ALL ADAPTERS ARE STOPPED (All V2X Hub plugins may remain enabled)"
    read -p "--> To continue, press [ENTER] " enter_press
    
    #if carma vehicle
    echo
    echo "Copying carma logs"
    latest_logs=$(readlink -f /opt/carma/logs/latest)
    cp -r $latest_logs .

    #if not a live vehicle (meaning must be constructive) then get adapter logs
    if [[ ! $vehicle_type == 'live_vehicle' ]]; then

        echo "Copying adapter logs"
        mv $VUG_ADAPTER_LOG_PATH/*.log .

        echo "Copying carma simulation logs"    
        mv $VUG_CARMA_SIM_LOG_PATH/*.log .
    
    fi

    echo
    read -p "Would you like to save the CARMA Platform rosbag? [y/n] " save_rosbag
    
    if [[ $save_rosbag =~ ^[yY]$ ]]; then
        echo "Copying latest bag file" 
        latest_bag=$(ls -t /opt/carma/logs/*.bag | head -1)
        cp $latest_bag .
    fi

    exit
}


echo
echo $j2735_tcpdump_out
echo $j2735_tcpdump_in
echo $j3224_tcpdump_out
echo $j3224_tcpdump_in
echo $collect_eco_cmd

echo
read -p "Data collection ready, press [ENTER] to begin..." dummyvar

echo
echo "Starting tcpdumps - when finished press [CTRL + C]"
echo 
$collect_eco_cmd & $j2735_tcpdump_out & $j2735_tcpdump_in & $j3224_tcpdump_out & $j3224_tcpdump_in & wait
