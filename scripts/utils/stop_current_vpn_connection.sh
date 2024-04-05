#!/usr/bin/bash

# Get output from openvpn3 sessions-list
text="$(sudo openvpn3 sessions-list)"
vpn_paths=()
path_regex='Path: (.*)'
vpn_statuses=()
status_regex='Status: (.*)'
has_stale_vpn_connections=false
has_active_vpn_connection=false
has_multiple_active_vpn_connections=false
while read line; do
    if [[ $line =~ $path_regex ]]
    then # Grab VPN path
        vpn_paths+=("${BASH_REMATCH[1]}")
    elif [[ $line =~ $status_regex ]]
    then # Grab VPN status
        if [[ ${BASH_REMATCH[1]} = "Connection, Client connected" ]]
        then
            if $has_active_vpn_connection
            then
                has_multiple_active_vpn_connections=true
            fi
            vpn_statuses+=(1)
            has_active_vpn_connection=true
        else
            vpn_statuses+=(0)
            has_stale_vpn_connections=true
        fi
    fi
done <<< "$text"

if [ $has_multiple_active_vpn_connections = false ] && [ $has_active_vpn_connection = true ]; then
    while true; do
        echo ''
        read -p "Would you like to end your VPN session? [y/N] " yn
        case $yn in
            [Yy]*) for i in $(seq 1 ${#vpn_statuses[@]}); do
                        if [ ${vpn_statuses[i-1]} = 1 ]
                        then
                            echo "Removing "${vpn_paths[i-1]}
                            sudo openvpn3 session-manage --session-path ${vpn_paths[i-1]} --disconnect
                        fi
                    done; break;;
            [Nn]* | "") echo ''; break;;
            * );;
        esac
    done
fi