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

xhost +local:docker

docker_compose_version=$(docker-compose -v || echo NA)

echo "docker-compose version: "$docker_compose_version

docker_compose_version_new=$(docker compose version || echo NA)

echo "docker compose version: "$docker_compose_version_new

if [ ! "$docker_compose_version_new" == "NA" ]; then
    docker_compose_cmd="docker compose"
else
    docker_compose_cmd="docker-compose"
fi

$docker_compose_cmd -f pilot2-event0_docker-compose.yml up

trap stopDocker SIGINT

stopDocker()
{

$docker_compose_cmd -f pilot2-event0_docker-compose.yml down
}