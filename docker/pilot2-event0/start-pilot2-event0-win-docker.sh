#!/bin/bash

stopDocker()
{

echo
echo STOPPING AND REMOVING VUG CONTAINERS
$docker_compose_cmd -f start-pilot2-event0-win-docker.sh down

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

        echo "Site Config: "$voices_site_config
        echo "Scenario Config: "$voices_scenario_config
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

trap stopDocker SIGINT

$docker_compose_cmd -f pilot2-event0-win_docker-compose.yml pull

$docker_compose_cmd -f pilot2-event0-win_docker-compose.yml up
