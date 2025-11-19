#!/bin/bash

stopDocker()
{

echo
echo STOPPING AND REMOVING VUG CONTAINERS
$docker_compose_cmd -f $docker_compose_file down
if [ $VUG_FORMAL_EVENT = true ]; then 
    source $VUG_LOCAL_DT_PATH/scripts/utils/stop_current_vpn_connection.sh
fi
}

# Get the directory of the script, no matter where it's called from
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd $SCRIPT_DIR

# Capture all extra arguments
EXTRA_ARGS=("$@")

dt_site_config=$HOME/.dt_site_config
dt_scenario_config=$HOME/.dt_scenario_config

if [ -L ${dt_site_config} ] && [ -L ${dt_scenario_config} ]; then
    if [ -e ${dt_site_config} ] && [ -e ${dt_scenario_config} ]; then
        site_config_link_dest=$(readlink -f $dt_site_config)
        site_link_base_name=$(basename ${site_config_link_dest})

        scenario_config_link_dest=$(readlink -f $dt_scenario_config)
        scenario_link_base_name=$(basename ${scenario_config_link_dest})

        source $dt_site_config
        source $dt_scenario_config
        export VUG_SITE_CONFIG_FILE=$site_link_base_name
        export VUG_SCENARIO_CONFIG_FILE=$scenario_link_base_name

        while true; do
			echo "Your current configs are:"
			echo "      Site Config: $site_link_base_name"
			echo "      Scenario Config:$scenario_link_base_name"
			echo
			read -p "Are these the configurations you would like to use? [Y/n] " yn
			case $yn in
				[Yy]* | "") break;;
				[Nn]*) exit 1;;
				* );;
			esac
		done
        
    else
        echo
        echo "[!!!] .dt_site_config or .dt_scenario_config link is broken"
        echo "Site Config: "$(readlink -f $dt_site_config)
        echo "Scenario Config: "$(readlink -f $dt_scenario_config)
        exit 1
   fi
elif [ -e ${dt_site_config} ] || [ -e ${dt_site_config} ]; then
    echo
    echo "[!!!] .dt_site_config or .dt_scenario_config file is not a symbolic link"
    echo "Site Config: "$(readlink -f $dt_site_config)
    echo "Scenario Config: "$(readlink -f $dt_scenario_config)
    exit 1
else
    echo
    echo "[!!!] .dt_site_config or .dt_scenario_config symbolic link does not exist"
    echo "Site Config: "$(readlink -f $dt_site_config)
    echo "Scenario Config: "$(readlink -f $dt_scenario_config)
    exit 1
fi

# Check if openvpn3 is installed
if ! command -v openvpn3 &> /dev/null
then
    if [ $VUG_FORMAL_EVENT = true ]
    then
        echo
        echo "openvpn3 is not installed but you have set VUG_FORMAL_EVENT=true in your scenario config."
        echo "Either install openvpn3 and activate a connection or set VUG_FORMAL_EVENT=false to continue."
        exit 1
    fi
    echo
    echo "openvpn3 could not be found. Skipping VPN checks..."
    # exit 0
fi

# Check if chronyc is installed
if ! command -v chronyc &> /dev/null
then
    if [ $VUG_FORMAL_EVENT = true ]
    then
        echo
        echo "chronyc is not installed but you have set VUG_FORMAL_EVENT=true in your scenario config."
        echo "Either install chronyc or set VUG_FORMAL_EVENT=false to continue."
        exit 1
    fi
    echo
    echo "chronyc could not be found. Skipping VPN checks..."
    # exit 0
fi

# Check if chronyc sources are present
if [ $VUG_FORMAL_EVENT = true ]
then
    chronyc_sources="$(chronyc sources)"
    valid_source=false
    while read -ra line; do
        if [[ ${line[4]} =~ ^-?[0-9]+$ ]] && [ ${line[4]} != 0 ]
        then
            valid_source=true
            break
        fi
    done <<< "$chronyc_sources"
    if [ $valid_source = false ]
    then
        echo
        echo 'There are no valid chronyc sources present'
        exit 1
    fi
fi

# Check if code is up to date with remote repository
if [ $VUG_FORMAL_EVENT = true ]
then
    git fetch
    if git status -uno | grep -q 'Your branch is behind'; then
        echo
        echo "Local repository is behind the remote repository. Please pull the most recent code."
        exit 1
    fi
fi

final_vpn_local_address=""
final_vpn_em_address=""


if [ $VUG_FORMAL_EVENT = true ]; then 

    # Checks for active openvpn3 sessions and:
    #   removes stale connections, 
    #   resolves accidentally connecting multiple times,
    #   checks if you are connected to VPN when you dont mean to be (FORMAL_EVENT=false), 
    #   makes sure you are connected when you mean to (FORMAL_EVENT=true)
    #
    # returns exit 1 if FORMAL_EVENT=true and no VPN connections found
    #
    # should result in 0 or 1 openvpn3 sessions
    if ! $VUG_LOCAL_DT_PATH/scripts/utils/prune_vpn_connections.sh; then
        exit 1
    fi

    # confirms we have openvpn3 session, if we do, prune_vpn_connections.sh ensures its the right one
    vpn_check=$(sudo openvpn3 sessions-list | grep -oE tun[0-9])
fi

# if we have an openvpn3 connection active, we want to be connecting to the portal
# go through some checks to automate and ensure IP information is correct
if [[ ! -z $vpn_check ]]; then

    interfaces=($(ip -o link show | awk -F': ' '{print $2}'))
    tun_interfaces=()
    tun_ip_addresses=()
    for i in $(seq 1 ${#interfaces[@]}); do
        if [ ${interfaces[i-1]:0:3} = "tun" ]
        then
            tun_interfaces+=(${interfaces[i-1]})
            vpn_local_ip=$(ip -br a show ${interfaces[i-1]} | awk '{print $3}')
            vpn_local_ip_clean=${vpn_local_ip%/*}
            tun_ip_addresses+=($vpn_local_ip_clean)
        fi
    done

    # checks the number of tun interfaces
    # this is different from the number of openvpn3 sessions,
    # as a user may have tun interfaces not from openvpn3

    # if there are more than one tun interfaces, prompt the user to pick one
    if [ "${#tun_interfaces[@]}" -gt 1 ]
    then
        echo
        echo "Multiple tunnel interfaces were found."
        for i in $(seq 1 ${#tun_interfaces[@]}); do
            echo "      " ${tun_interfaces[i-1]} ${tun_ip_addresses[i-1]}
        done
        while true; do
            echo
            read -p "Which interface would you like to use? [0-9] " tun
            case $tun in
                [0-9])  vpn_check="tun$tun"; break;;
                * );;
            esac
        done
    # if there is only one tun interface found, make sure it is the one we want
    elif [ "${#tun_interfaces[@]}" = 1 ]
    then
        echo
        echo "A VPN tunnel interface was found."
        for i in $(seq 1 ${#tun_interfaces[@]}); do
            echo "      " ${tun_interfaces[i-1]} ${tun_ip_addresses[i-1]}
        done
        while true; do
            echo
            read -p "Is this the correct interface for distributed testing? [Y/n] " yn
            case $yn in
                [Yy]* | "")  vpn_check=${tun_interfaces[0]} break;;
                [Nn]*) exit 1;;
                * );;
            esac
        done
    # otherwise we have no tunnel interfaces found, which we need 
    else
        echo 
        echo "OpenVPN3 connection active but no tunnel interface found."
        echo "Please try disconnecting and reconnecting to the VPN."
        exit 1

    fi

    # get the IP from the tunnel interface we found and clean it
    vpn_local_ip=$(ip -br a show $vpn_check | awk '{print $3}')
    vpn_local_ip_clean=${vpn_local_ip%/*}

    # check to see if the VPN IP we got was valid
    # if not, prompt the user to enter it manually
    if [[ ! "$vpn_local_ip_clean" =~ ^(([1-9]?[0-9]|1[0-9][0-9]|2([0-4][0-9]|5[0-5]))\.){3}([1-9]?[0-9]|1[0-9][0-9]|2([0-4][0-9]|5[0-5]))$ ]]; then
        echo
        echo "Unable to automatically get VPN address for this machine"
        read -p "Please enter your VPN IP address (found under the tun interface using the command: 'ip -br a show' ) [###.###.###.###]: " manual_vpn_local_ip

        # check to see if the manually entered IP is valid, if not, exit
        if [[ "$manual_vpn_local_ip" =~ ^(([1-9]?[0-9]|1[0-9][0-9]|2([0-4][0-9]|5[0-5]))\.){3}([1-9]?[0-9]|1[0-9][0-9]|2([0-4][0-9]|5[0-5]))$ ]]; then
            echo "    IP $manual_vpn_local_ip is a valid IP"
            vpn_local_ip_clean=$manual_vpn_local_ip
        else
            echo
            echo "    IP $manual_vpn_local_ip is NOT valid, please find your VPN address and try again"
            vpn_local_ip_clean=""
            exit
        fi
    fi
fi

# check the configured EM address to see if it is not in IP format
# if it is not in IP format, assume its a hostname and try to get IP from that
# echo $VUG_EM_ADDRESS
# echo $VUG_VPN_EM_ADDRESS
if [[ ! "$VUG_EM_ADDRESS" =~ ^(([1-9]?[0-9]|1[0-9][0-9]|2([0-4][0-9]|5[0-5]))\.){3}([1-9]?[0-9]|1[0-9][0-9]|2([0-4][0-9]|5[0-5]))$ ]] && [[ ! "$VUG_EM_ADDRESS" == 'localhost' ]]; then

    # get the IP of the entered EM hostname
    em_fqdn_address=$(getent hosts $VUG_EM_ADDRESS | awk '{print $1}')
    
    
    # if the retrieved EM IP is valid, use it
    if [[ "$em_fqdn_address" =~ ^(([1-9]?[0-9]|1[0-9][0-9]|2([0-4][0-9]|5[0-5]))\.){3}([1-9]?[0-9]|1[0-9][0-9]|2([0-4][0-9]|5[0-5]))$ ]]; then

        echo
        echo "Automatically detected EM from hostname $VUG_EM_ADDRES: $em_fqdn_address"

    # if the retrieved EM IP is not valid, prompt to enter manually
    else
    
        echo
        echo "Unable to automatically get EM address"
        read -p "Please enter the VPN EM Address (found in the VOICES Portal under Connection Information) [###.###.###.###]: " manual_vpn_em_address

        # if the entered EM IP is valid, use it, otherwise exit
        if [[ "$manual_vpn_em_address" =~ ^(([1-9]?[0-9]|1[0-9][0-9]|2([0-4][0-9]|5[0-5]))\.){3}([1-9]?[0-9]|1[0-9][0-9]|2([0-4][0-9]|5[0-5]))$ ]]; then
            echo "    IP $manual_vpn_em_address is a valid IP"
            em_fqdn_address=$manual_vpn_em_address
        else
            echo
            echo "    IP $manual_vpn_em_address is NOT valid, please find your VPN address and try again"
            em_fqdn_address=""
            exit
        fi
    fi
fi

# these environment vars get imported from the docker compose
# if they are set, they override the VUG_LOCAL_ADDRESS and VUG_EM_ADDRESS
export VUG_VPN_LOCAL_ADDRESS=$vpn_local_ip_clean
export VUG_VPN_EM_ADDRESS=$em_fqdn_address

echo
xhost +local:docker

docker_compose_v2_version=$(docker compose version 2> /dev/null)

if [ ! -z "$docker_compose_v2_version" ]; then
    echo
    echo "docker compose version: "$docker_compose_v2_version

    docker_compose_cmd="docker compose"
else
    docker_compose_v1_version=$(docker-compose -v 2> /dev/null)

    if [ ! -z "$docker_compose_v1_version" ]; then

        echo
        echo "docker-compose version: "$docker_compose_v1_version

        docker_compose_cmd="docker-compose"
    else
        echo
        echo ERROR: No valid docker compose version found
        exit
    fi

fi

echo
if [[ $VUG_DOCKER_START_CARLA == 'local' ]]; then
    echo "Using CARLA docker-compose"
    docker_compose_file='docker-compose.yml'
else
    echo "Using NO CARLA docker-compose"
    docker_compose_file='no-carla_docker-compose.yml'
fi


trap stopDocker SIGINT

# $docker_compose_cmd -f $docker_compose_file pull

$docker_compose_cmd -f $docker_compose_file up "${EXTRA_ARGS[@]}"