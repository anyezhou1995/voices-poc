#!/bin/bash

docker_compose_file='pilot2-event2_docker-compose.yml'

stopDocker()
{

echo
echo STOPPING AND REMOVING VUG CONTAINERS
$docker_compose_cmd -f $docker_compose_file down
source $VUG_LOCAL_VOICES_POC_PATH/scripts/utils/stop_current_vpn_connection.sh
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
        export VUG_SITE_CONFIG_FILE=$site_link_base_name
        export VUG_SCENARIO_CONFIG_FILE=$scenario_link_base_name

        while true; do
			echo "Your current configs are:"
			echo "      Site Config: $site_link_base_name"
			echo "      Scenario Config:$scenario_link_base_name"
			echo
			read -p "Would you like to continue? [Y/n] " yn
			case $yn in
				[Yy]* | "") break;;
				[Nn]*) exit 1;;
				* );;
			esac
		done
        
    else
        echo
        echo "[!!!] .voices_site_config or .voices_scenario_config link is broken"
        echo "Site Config: "$(readlink -f $voices_site_config)
        echo "Scenario Config: "$(readlink -f $voices_scenario_config)
        exit 1
   fi
elif [ -e ${voices_site_config} ] || [ -e ${voices_site_config} ]; then
    echo
    echo "[!!!] .voices_site_config or .voices_scenario_config file is not a symbolic link"
    echo "Site Config: "$(readlink -f $voices_site_config)
    echo "Scenario Config: "$(readlink -f $voices_scenario_config)
    exit 1
else
    echo
    echo "[!!!] .voices_site_config or .voices_scenario_config symbolic link does not exist"
    echo "Site Config: "$(readlink -f $voices_site_config)
    echo "Scenario Config: "$(readlink -f $voices_scenario_config)
    exit 1
fi

# Conduct VPN connectivity checks
if ! $VUG_LOCAL_VOICES_POC_PATH/scripts/utils/prune_vpn_connections.sh; then
    exit 1
fi

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

final_vpn_local_address=""
final_vpn_em_address=""

vpn_check=$(sudo openvpn3 sessions-list | grep -oE tun[0-9])

interfaces=($(ifconfig -a | grep -o '^[^ ]\+'))
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
# Prompt if there are multiple tun interfaces
if [ "${#tun_interfaces[@]}" -gt 1  ]
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
elif [ "${#tun_interfaces[@]}" = 1  ]
then
    echo
    echo "A single tunnel interface was found."
    for i in $(seq 1 ${#tun_interfaces[@]}); do
        echo "      " ${tun_interfaces[i-1]} ${tun_ip_addresses[i-1]}
    done
    while true; do
        echo
        read -p "Is this the correct interface? [Y/n] " yn
        case $yn in
            [Yy]* | "")  vpn_check=${tun_interfaces[0]} break;;
            [Nn]*) exit 1;;
            * );;
        esac
    done
fi

if [[ ! -z $vpn_check ]]; then
    echo
    echo "VPN interface found: $vpn_check"

    vpn_local_ip=$(ip -br a show $vpn_check | awk '{print $3}')
    vpn_local_ip_clean=${vpn_local_ip%/*}

    if [[ "$vpn_local_ip_clean" =~ ^(([1-9]?[0-9]|1[0-9][0-9]|2([0-4][0-9]|5[0-5]))\.){3}([1-9]?[0-9]|1[0-9][0-9]|2([0-4][0-9]|5[0-5]))$ ]]; then
        echo "    IP $vpn_local_ip_clean is a valid IP"
    else
        echo
        echo "Unable to automatically get VPN local address"
        read -p "Please enter your local IP address for the VPN (found under the tun interface using the command: 'ip -br a show' ) [###.###.###.###]: " manual_vpn_local_ip

        if [[ "$manual_vpn_local_ip" =~ ^(([1-9]?[0-9]|1[0-9][0-9]|2([0-4][0-9]|5[0-5]))\.){3}([1-9]?[0-9]|1[0-9][0-9]|2([0-4][0-9]|5[0-5]))$ ]]; then
            echo "    IP $manual_vpn_local_ip is a valid IP"
            vpn_local_ip_clean=$manual_vpn_local_ip
        else
            echo
            echo "    IP $manual_vpn_local_ip is NOT valid, please set VUG_LOCAL_ADDRESS manually in your site config"
            vpn_local_ip_clean=""
        fi
    fi


    em_fqdn_address=$(getent hosts em.voices-network.local | awk '{print $1}')

    if [[ "$em_fqdn_address" =~ ^(([1-9]?[0-9]|1[0-9][0-9]|2([0-4][0-9]|5[0-5]))\.){3}([1-9]?[0-9]|1[0-9][0-9]|2([0-4][0-9]|5[0-5]))$ ]]; then
        echo
        echo "Automatically detected EM at $em_fqdn_address"
    else
        echo
        echo "Unable to automatically get EM address"
        read -p "Please enter the VPN EM Address (found in the VOICES Portal under Connection Information) [###.###.###.###]: " manual_vpn_em_address

        if [[ "$manual_vpn_em_address" =~ ^(([1-9]?[0-9]|1[0-9][0-9]|2([0-4][0-9]|5[0-5]))\.){3}([1-9]?[0-9]|1[0-9][0-9]|2([0-4][0-9]|5[0-5]))$ ]]; then
            echo "    IP $manual_vpn_em_address is a valid IP"
            em_fqdn_address=$manual_vpn_em_address
        else
            echo
            echo "    IP $manual_vpn_em_address is NOT valid, please set VUG_LOCAL_ADDRESS manually in your site config"
            em_fqdn_address=""
        fi
    fi
fi

export VUG_VPN_LOCAL_ADDRESS=$vpn_local_ip_clean
export VUG_VPN_EM_ADDRESS=$em_fqdn_address

echo

trap stopDocker SIGINT

$docker_compose_cmd -f $docker_compose_file pull

$docker_compose_cmd -f $docker_compose_file up
