#!/bin/bash

docker_compose_file='pilot2-event2_no-carla_docker-compose.yml'

stopDocker()
{

echo
echo STOPPING AND REMOVING VUG CONTAINERS
$docker_compose_cmd -f $docker_compose_file down

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

xhost +local:docker

docker_compose_v2_version=$(docker compose version 2> /dev/null)

if [ ! -z "$docker_compose_v2_version" ]; then

    echo "docker compose version: "$docker_compose_v2_version
    
    docker_compose_cmd="docker compose"
else
    docker_compose_v1_version=$(docker-compose -v 2> /dev/null)

    if [ ! -z "$docker_compose_v1_version" ]; then
        
        echo "docker-compose version: "$docker_compose_v1_version

        docker_compose_cmd="docker-compose"
    else
        echo ERROR: No valid docker compose version found
        exit
    fi
    
fi

final_vpn_local_address=""
final_vpn_em_address=""

vpn_interface_pattern="tun[0-9]"
vpn_check=$(ip -br link show | awk '{print $1}' | grep -w "$vpn_interface_pattern")

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
    
    if [[ ! -z $vpn_local_ip_clean ]]; then

        echo
        read -p "Would you like to use the VPN IP as VUG_LOCAL_ADDRESS? ($vpn_local_ip_clean) [y/n] " use_local_vpn_ip
    
        if [[ $use_local_vpn_ip =~ ^[yY]$ ]]; then

            final_vpn_local_address=$vpn_local_ip_clean
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

    if [[ ! -z $em_fqdn_address ]]; then

        echo
        read -p "Would you like to use the vpn IP as VUG_EM_ADDRESS? ($em_fqdn_address) [y/n] " use_em_vpn_ip
    
        if [[ $use_em_vpn_ip =~ ^[yY]$ ]]; then

            final_vpn_em_address=$em_fqdn_address
        fi

    fi

fi

export VUG_VPN_LOCAL_ADDRESS=$final_vpn_local_address
export VUG_VPN_EM_ADDRESS=$final_vpn_em_address

echo

trap stopDocker SIGINT

$docker_compose_cmd -f $docker_compose_file pull

$docker_compose_cmd -f $docker_compose_file up
