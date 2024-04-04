#! /bin/bash

export HOME=/home

env_set_site_config_path=$HOME/voices-poc/config/site_config/$VUG_SITE_CONFIG_FILE
env_set_scenario_config_path=$HOME/voices-poc/config/scenario_config/$VUG_SCENARIO_CONFIG_FILE

# the .voices_config setup is as follows in order to make existing scripts work within docker container: 
# 
# $HOME/.voices_site_config_link = actual site config link
# $HOME/.voices_scenario_config_link = actual scenario config link
# 
# 
# .voices_site_config_docker = source $HOME/.voices_site_config_link + overwrite vars pertaining to locations within docker container
# .voices_scenario_config_docker = source $HOME/.voices_scenario_config_link + overwrite vars pertaining to locations within docker container
# 
# therefore:
# 
# $HOME/.voices_site_config --> $HOME/.voices_site_config_docker = source $HOME/.voices_site_config_link + overwrites
# $HOME/.voices_scenario_config --> $HOME/.voices_scenario_config_docker = source $HOME/.voices_scenario_config_link + overwrites
# 
# $HOME/.voices_site_config_docker and $HOME/.voices_site_config_docker exist because $HOME/.voices_site_config and $HOME/.voices_scenario_config must be a sym link for run_scripts
# 

export SUMO_HOME=/usr/share/sumo

if [ ! -f $env_set_site_config_path ]; then
        echo "    [!!!] Site Config file not found: $env_set_site_config_path"
        exit 1
elif [ ! -f $env_set_scenario_config_path ]; then
        echo "    [!!!] Scenario Config file not found: $env_set_scenario_config_path"
        exit 1
else
        ln -sf $env_set_site_config_path $HOME/.voices_site_config_link
        ln -sf $env_set_scenario_config_path $HOME/.voices_scenario_config_link
      #   ln -sf $HOME/.voices_site_config_docker $HOME/.voices_site_config
      #   ln -sf $HOME/.voices_scenario_config_docker $HOME/.voices_scenario_config
fi

# voices_site_config=$HOME/.voices_site_config_link
# voices_scenario_config=$HOME/.voices_scenario_config_link

voices_site_config=$HOME/.voices_site_config
voices_scenario_config=$HOME/.voices_scenario_config

ln -sf $HOME/.voices_site_config_link $voices_site_config
ln -sf $HOME/.voices_scenario_config_link $voices_scenario_config

voices_site_config_docker=$HOME/.voices_site_config_docker
voices_scenario_config_docker=$HOME/.voices_scenario_config_docker

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
      echo "[!!!] .voices_config link is broken"
      exit 1
   fi
elif [ -e ${voices_site_config} ] || [ -e ${voices_site_config} ]; then
   echo "[!!!] .voices_config file is not a symbolic link"
   exit 1
else
   echo "[!!!] .voices_config link is is missing"
   exit 1
fi

