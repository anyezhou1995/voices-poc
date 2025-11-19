#! /bin/bash

# how long to wait before force kill
: "${GRACE_SECONDS:=20}"

# optional: patterns to include/exclude
MATCH_RE='VUG_Adapters|distributed-testing'
EXCLUDE_RE='executionMan'

descendants() {
  local root="${1:-$$}"
  # all descendants of PID $root
  pgrep -P "$root" || true
  for pid in $(pgrep -P "$root" || true); do
    descendants "$pid"
  done
}

cleanup() {
  (( CLEANUP_RAN )) && return
  CLEANUP_RAN=1

  echo "Stopping TENA Applications Gracefully"

  # Let dumb-init finish first to avoid racing (200ms)
  sleep 0.2

  # 1) Prefer killing ONLY what we started (descendants of this shell)
  mapfile -t PIDS < <(descendants $$ | sort -u)

  # 2) Filter by include/exclude patterns, and skip ourselves
  TARGET=()
  for pid in "${PIDS[@]}"; do
    [[ -z "$pid" || "$pid" -eq "$$" ]] && continue
    cmd=$(ps -p "$pid" -o cmd= 2>/dev/null) || continue
    [[ -z "$cmd" ]] && continue
    [[ "$cmd" =~ $MATCH_RE ]] || continue
    [[ "$cmd" =~ $EXCLUDE_RE ]] && continue
    TARGET+=("$pid")
  done

  # 3) Fallback sweep: if nothing was found but something might still be up
  if ((${#TARGET[@]} == 0)); then
    mapfile -t TARGET < <(
      pgrep -f "$MATCH_RE" | while read -r pid; do
        cmd=$(ps -p "$pid" -o cmd= 2>/dev/null) || continue
        [[ "$cmd" =~ $EXCLUDE_RE ]] && continue
        echo "$pid"
      done
    )
  fi

  if ((${#TARGET[@]} == 0)); then
    echo "No matching processes found."
    return
  fi

  # Ask nicely, then wait, then force
  for pid in "${TARGET[@]}"; do
    echo "TERM -> PID $pid  $(ps -p "$pid" -o cmd=)"
  done
  kill -TERM "${TARGET[@]}" 2>/dev/null || true

  end=$((SECONDS + GRACE_SECONDS))
  while (( SECONDS < end )); do
    alive=()
    for pid in "${TARGET[@]}"; do
      kill -0 "$pid" 2>/dev/null && alive+=("$pid")
    done
    ((${#alive[@]}==0)) && break
    sleep 1
  done

  if ((${#alive[@]:-0} > 0)); then
    for pid in "${alive[@]}"; do
      echo "KILL -> PID $pid  $(ps -p "$pid" -o cmd=)"
      kill -KILL "$pid" 2>/dev/null || true
    done
  fi
}

trap cleanup TERM INT

source $HOME/start_scripts/setup-docker.sh

if [[ $VUG_DEV_MODE == true ]]; then
   echo "DEV MODE ENABLED, PLEASE RUN START SCRIPT MANUALLY"
   exit 0
fi

sleep 5s

if [[ $VUG_DOCKER_START_EM == true ]]; then
   echo "STARTING TENA EXECUTION MANAGER"
   $HOME/distributed-testing/scripts/run_scripts/start-em-bg.sh

   sleep 5s
fi

if [[ $VUG_DOCKER_START_CONSOLE == true ]]; then
   echo "STARTING TENA CONSOLE"
   $VUG_LOCAL_TENA_PATH/Console-v2.0.1/start.sh -emEndpoints $VUG_EM_ADDRESS:$VUG_EM_PORT -listenEndpoints $VUG_LOCAL_ADDRESS -autoConnect &

   sleep 5s
fi

# try to change carla map 
if [[ $VUG_DOCKER_START_CARLA == "local" ]] || [[ $VUG_DOCKER_START_CARLA == "remote" ]]; then
   echo "CHANGING CARLA MAP TO: $VUG_CARLA_MAP_NAME"
	python3 $HOME/distributed-testing/scripts/carla_python_scripts/config.py \
      -m $VUG_CARLA_MAP_NAME --weather ClearNoon --host $VUG_CARLA_ADDRESS 2>&1 | awk '{ print "CHANGE MAP: ", $0; fflush(); }'

   sleep 5s
   
   if [[ $VUG_CARLA_BLANK_SIGNALS == true ]]; then
      # blank signals and essentially disable their timing so that the only TL states we see are from TENA TrafficLight SDO updates
      python3 "$HOME/distributed-testing/scripts/carla_python_scripts/blank_traffic_signals.py" \
         --host "$VUG_CARLA_ADDRESS" 2>&1 | awk '{ print "BLANK SIGNALS: ", $0; fflush(); }'
   fi

   # set spectator view
   if [[ $VUG_CARLA_MAP_NAME == *"mcity"* ]]; then
      python3 $HOME/distributed-testing/scripts/carla_python_scripts/spectator_view_mcity.py --host $VUG_CARLA_ADDRESS 2>&1 | awk '{ print "SET VIEW: ", $0; fflush(); }'
   fi

   # set spectator view
   if [[ $VUG_CARLA_MAP_NAME == *"Delave"* ]]; then
      python3 $HOME/distributed-testing/scripts/carla_python_scripts/spectator_view_delave.py --host $VUG_CARLA_ADDRESS 2>&1 | awk '{ print "SET VIEW: ", $0; fflush(); }'
   fi
  

   if [[ $VUG_DISPLAY_VEHICLE_ROLENAMES == true ]] || [[ $VUG_DISPLAY_TRAFFIC_SIGNAL_STATES == true ]]; then
      # display vehicle names and/or traffic light info
      python3 $HOME/distributed-testing/scripts/carla_python_scripts/display_carla_info.py --host $VUG_CARLA_ADDRESS -d 0 2>&1 | awk '{ print "CARLA INFO: ", $0; fflush(); }' &
   fi

   if [[ $VUG_DISPLAY_SDSM == true ]]; then
      # display SDSMs as they are received
      python3 $HOME/distributed-testing/scripts/carla_python_scripts/draw_sdsm_json_live.py --host $VUG_CARLA_ADDRESS 2>&1 | awk '{ print "DISPLAY SDSM: ", $0; fflush(); }' &
   fi

   sleep 5s
fi

if [[ $VUG_DOCKER_START_SUMO == true ]]; then
   echo "STARTING SUMO"
   cd $HOME/distributed-testing/scripts/carla_python_scripts/Sumo/
   python3 $HOME/distributed-testing/scripts/carla_python_scripts/Sumo/run_synchronization.py $HOME/distributed-testing/scripts/carla_python_scripts/Sumo/$VUG_DOCKER_SUMO_CONFIG --sumo-gui --tls-manager carla --carla-host $VUG_LOCAL_ADDRESS --sumo-host $VUG_LOCAL_ADDRESS &
   cd $HOME
   sleep 5s
fi

if [[ $VUG_DOCKER_START_CANARY == true ]]; then
   echo "STARTING TENA CANARY"
   $VUG_LOCAL_TENA_PATH/tenaCanary-v1.0.13/start.sh -emEndpoints $VUG_EM_ADDRESS:$VUG_EM_PORT -listenEndpoints $VUG_LOCAL_ADDRESS -auto 2>&1 | awk '{ print "TENA CANARY: ", $0; fflush(); }' &

   sleep 5s
fi

if [[ $VUG_DOCKER_START_TDCS == true ]]; then
   echo "STARTING TENA DATA COLLECTION SYSTEM"
   $HOME/distributed-testing/scripts/run_scripts/start-tdcs.sh 2>&1 | awk '{ print "TENA COLLECTOR: ", $0; fflush(); }' &
   
   sleep 5s
fi

if [[ $VUG_DOCKER_START_TENA_PLAYBACK == true ]]; then
   echo "STARTING TENA PLAYBACK SYSTEM"
   $HOME/distributed-testing/scripts/run_scripts/start-playback-tool.sh 2>&1 | awk '{ print "TENA PLAYBACK: ", $0; fflush(); }' &
   
   sleep 5s
fi

if [[ $VUG_DOCKER_START_DATAVIEW == true ]]; then
   echo "STARTING TENA DATAVIEW"
   $VUG_LOCAL_TENA_PATH/DataView-v1.5.4/start.sh 2>&1 | awk '{ print "DATAVIEW: ", $0; fflush(); }' &

   sleep 5s
fi

if [[ $VUG_DOCKER_START_SCENARIO_PUBLISHER == true ]]; then
   echo "STARTING SCENARIO PUBLISHER"
   $HOME/distributed-testing/scripts/run_scripts/start-scenario-publisher.sh &

   sleep 5s
fi

if [[ $VUG_DOCKER_START_TENA_CARLA_ADAPTER == true ]]; then
   echo "STARTING TENA CARLA ADAPTER"
   $HOME/distributed-testing/scripts/run_scripts/start-carla-tena-adapter.sh &

   sleep 5s
fi

if [[ $VUG_DOCKER_START_JSON_STREAMER == true ]]; then
   echo "STARTING JSON STREAMER"
   $HOME/distributed-testing/scripts/run_scripts/start-json-streamer.sh &

   sleep 5s
fi

if [[ $VUG_DOCKER_START_JSON_PUBLISHER == true ]]; then
   echo "STARTING JSON PUBLISHER"
   $HOME/distributed-testing/scripts/run_scripts/start-json-publisher.sh &

   sleep 5s
fi


if [[ $VUG_DOCKER_START_V2X_ADAPTER == true ]]; then
   echo "STARTING TENA V2X ADAPTER"
   $HOME/distributed-testing/scripts/run_scripts/start-tv2x-adapter.sh &

   sleep 5s
fi

if [[ $VUG_DOCKER_START_TRAFFIC_LIGHT_EG == true ]]; then
   echo "STARTING TENA TRAFFIC LIGHT ENTITY GENERATOR"
   $HOME/distributed-testing/scripts/run_scripts/start-traffic-light-entity-generator.sh &

   sleep 5s
fi

if [[ $VUG_DOCKER_START_ENTITY_GENERATOR == true ]]; then
   echo "STARTING TENA ENTITY GENERATOR"
   $HOME/distributed-testing/scripts/run_scripts/start-entity-generator.sh &

   sleep 5s
fi

if [[ $VUG_DOCKER_START_MANUAL_CARLA_VEHICLE == true ]]; then
   echo "STARTING MANUAL CARLA VEHICLE"

   # we have migrated to having all participants spawn in their own vehicles. the CARLA Adapter will no longer spawn in vehicles for people
   # therefore we will always create a vehicle
   echo "   SPAWNING NEW MANUAL VEHICLE"
   python3 $HOME/distributed-testing/scripts/carla_python_scripts/manual_control_keyboard.py --rolename $VUG_MANUAL_VEHICLE_ID --host $VUG_CARLA_ADDRESS --speed_limit $VUG_MANUAL_VEHICLE_SPEED_LIMIT \
   --x $VUG_MANUAL_VEHICLE_SPAWN_X \
   --y $VUG_MANUAL_VEHICLE_SPAWN_Y \
   --z $VUG_MANUAL_VEHICLE_SPAWN_Z \
   --roll $VUG_MANUAL_VEHICLE_SPAWN_ROLL \
   --pitch $VUG_MANUAL_VEHICLE_SPAWN_PITCH \
   --yaw $VUG_MANUAL_VEHICLE_SPAWN_YAW \
   --filter "${VUG_MANUAL_VEHICLE_MODEL}*"
   2>&1 | awk '{ print "MANUAL VEHICLE: ", $0; fflush(); }' &

   sleep 5s
fi

echo
echo "VUG STARTUP COMLPETE"

# Keep PID 1 alive and responsive to TERM/INT
# Wait for first child to exit, then for the rest; traps will run cleanup.
wait -n || true
wait || true

cleanup