#!/usr/bin/env python3
import time
import json
import socket
import os
from datetime import datetime
import argparse
from argparse import RawTextHelpFormatter

import j2735_202409

MessageFrame = j2735_202409.MessageFrame.MessageFrame


def compute_moy_and_time_mark():
    """
    Compute:
      - Minute of year (moy)
      - TimeMark in 0.1s units from the top of the current UTC hour (0..35999)
    """
    now = datetime.utcnow()

    # Minute of year (unchanged)
    moy = ((now.timetuple().tm_yday - 1) * 24 * 60) + now.hour * 60 + now.minute

    # Seconds since top of the hour
    seconds_since_hour = now.minute * 60 + now.second
    ms_since_hour = seconds_since_hour * 1000 + now.microsecond // 1000

    # TimeMark: 0.1 s units from top of the current hour, should be 0..35999
    time_mark = ms_since_hour // 100
    if time_mark > 35999:
        # Should not normally happen, but be safe
        time_mark = 35999

    return moy, int(time_mark)




def build_spat_for_intersection(
    intersection_id,
    intersection_ip,
    moy,
    time_mark,
    signal_groups,
):
    """
    Build a SPaT JER dict for a single intersection, given existing timing/state info.
    """

    """
    get signal group states from TSC
    """
    def get_signal_group_state(signal_group):
        
        ### configure your OIDs and port here
        SIGNAL_GROUP_STATE_OID = "0.1.2.3.4.5"
        MIN_END_TIME_OID = "0.1.2.3.4.5"
        MAX_END_TIME_OID = "0.1.2.3.4.5"
        NTCIP_PORT = 1000

        # event_state = fake_ntcip.get(intersection_ip,NTCIP_PORT,SIGNAL_GROUP_STATE_OID)
        # min_end_time = fake_ntcip.get(intersection_ip,NTCIP_PORT,SIGNAL_GROUP_STATE_OID)
        # max_end_time = fake_ntcip.get(intersection_ip,NTCIP_PORT,SIGNAL_GROUP_STATE_OID)

        event_state = "permissive-Movement-Allowed"
        min_end_time = 5000
        max_end_time = 5000

        ### IF THERE IS ANY CONVERSION THAT NEEDS TO BE DONE TO GO FROM NTCIP TO J2725 FORMAT, DO IT HERE
        # 
        # TimeMark ::= INTEGER (0..36111)
        # -- In units of 1/10th second from UTC time in the current or next hour
        # -- A range of 0~35999 covers one hour
        # -- The values 36000..36009 are used when a leap second occurs
        # -- The values 36010..36110 are reserved for future use
        # -- 36111 is to be used when the value is undefined or unknown
        # -- Note that this is NOT expressed in GPS time or in local time
        # 3:40
        # MovementPhaseState ::= ENUMERATED {
        # -- Note that based on the regions and the operating mode not every 
        # -- phase will be used in all transportation modes and that not 
        # -- every phase will be used in all transportation modes
        
        # unavailable (0), 
        # -- This state is used for unknown or error 
        # dark (1), 
        # -- The signal head is dark (unlit)
        # -- Reds
        # stop-Then-Proceed (2), 
        # -- Often called 'flashing red' in US
        # -- Driver Action: 
        # -- Stop vehicle at stop line. 
        # -- Do not proceed unless it is safe.
        # -- Note that the right to proceed either right or left when 
        # -- it is safe may be contained in the lane description to 
        # -- handle what is called a 'right on red' 
        # stop-And-Remain (3),
        # -- e.g., called 'red light' in US
        # -- Driver Action: 
        # -- Stop vehicle at stop line. 
        # -- Do not proceed. 
        # -- Note that the right to proceed either right or left when 
        # -- it is safe may be contained in the lane description to 
        # -- handle what is called a 'right on red' 
        
        # -- Greens
        # pre-Movement (4), 
        # -- Not used in the US, red+yellow partly in EU
        # -- Driver Action: 
        # -- Stop vehicle. 
        # -- Prepare to proceed (pending green)
        # -- (Prepare for transition to green/go)
        # permissive-Movement-Allowed (5), 
        # -- Often called 'permissive green' in US
        # -- Driver Action: 
        # -- Proceed with caution, 
        # -- must yield to all conflicting traffic 
        # -- Conflicting traffic may be present
        # -- in the intersection conflict area
        # protected-Movement-Allowed (6), 
        # -- Often called 'protected green' in US
        # -- Driver Action: 
        # -- Proceed, tossing caution to the wind, 
        # -- in indicated (allowed) direction.
        
        # -- Yellows/Ambers
        # -- The vehicle is not allowed to cross the stop bar if it is possible 
        # -- to stop without danger. 
        # permissive-clearance (7), 
        # -- Often called 'permissive yellow' in US
        # -- Driver Action: 
        # -- Prepare to stop.
        # Downloaded from SAE International by Andrew Loughran, Friday, December 15, 2023
        # SAE INTERNATIONAL J2735® SEP2023 Page 170 of 275
        # -- Proceed if unable to stop,
        # -- Clear Intersection.
        # -- Conflicting traffic may be present
        # -- in the intersection conflict area
        # protected-clearance (8), 
        # -- Often called 'protected yellow' in US
        # -- Driver Action: 
        # -- Prepare to stop.
        # -- Proceed if unable to stop,
        # -- in indicated direction (to connected lane)
        # -- Clear Intersection.
        
        # caution-Conflicting-Traffic (9)
        # -- Often called 'flashing yellow' in US
        # -- Often used for extended periods of time
        # -- Driver Action: 
        # -- Proceed with caution, 
        # -- Conflicting traffic may be present
        # -- in the intersection conflict area
        # } 
        # -- The above number assignments are not used with UPER encoding
        # -- and are only to be used with DER or implicit encoding

        return {
            "signalGroup": signal_group,
            "state-time-speed": [
                {
                    "eventState": event_state,
                    "timing": {
                        # Both are INTEGER TimeMark values
                        "minEndTime": min_end_time,
                        "maxEndTime": max_end_time,
                    },
                }
            ],
        }

    states = []
    # Signal groups
    for sg in signal_groups:
        states.append(get_signal_group_state(sg))

    ### this gets time from the local clock, if you want to get time from the controller, 
    #   you will need to change the time stamps here to get from NTCIP 

    spat = {
        "messageId": 19,
        "value": {
            "timeStamp": int(time_mark),  # DSecond-ish; still 0.1s from hour, but valid INTEGER
            "intersections": [
                {
                    "id": {"id": int(intersection_id)},
                    "revision": 0,
                    "status": "0000",
                    "moy": int(moy),
                    "timeStamp": int(time_mark),
                    "states": states,
                }
            ],
        },
    }

    return spat


def encode_spat_to_uper_hex(spat_dict):
    jer_str = json.dumps(spat_dict, separators=(",", ":"))
    msg = MessageFrame
    msg.from_jer(jer_str)
    uper_bytes = msg.to_uper()
    return uper_bytes.hex()


def build_active_message(hex_payload):
    """
    Build Active Message Format (AMF) text with the given hex payload.
    """
    return (
        "Version=0.7\n"
        "Type=SPAT\n"
        "PSID=0x8003\n"
        "Priority=7\n"
        "TxMode=CONT\n"
        "TxChannel=183\n"
        "TxInterval=0\n"
        "DeliveryStart=\n"
        "DeliveryStop=\n"
        "Signature=True\n"
        "Encryption=False\n"
        f"Payload={hex_payload}\n"
    )


def print_frame_log(intersection_id, debug_info, hex_str):
    print(
        f"[SPaT] Int {intersection_id} | "
        f"{debug_info['timestamp']} | "
        f"Signal groups={debug_info['signal_groups']}\n"
        f"   HEX: {hex_str}\n"
    )


def load_phase_config(path):
    """
    Load phase group configuration from JSON.

    Expected JSON format:
      {
        "intersections": [
          {"id": 100, "signal_groups": [10,12,14],"ip": "1.2.3.4"},
          {"id": 101, "signal_groups": [2,6,8],"ip": "1.2.3.5"}
        ]
      }
    """
    if not path:
        raise ValueError("A config file is required; provide --config <path>")

    path = os.path.expanduser(path)
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    intersections = []

    if isinstance(data, dict):
        if "intersections" in data and isinstance(data["intersections"], list):
            for item in data["intersections"]:
                if not isinstance(item, dict) or "id" not in item:
                    continue
                signal_groups = item.get("signal_groups")
                if not signal_groups:
                    raise ValueError(f"Intersection {item.get('id')} missing signal_groups")
                intersections.append(
                    {
                        "id": int(item["id"]),
                        "signal_groups": signal_groups,
                        "ip": item.get("ip"),
                    }
                )

    if not intersections:
        raise ValueError("No intersections defined in config; each needs signal_groups")

    return intersections


def main():
    parser = argparse.ArgumentParser(
        formatter_class=RawTextHelpFormatter, description=(
            "Generate SPaT using signal group states polled from an NTCIP TSC and send as "
            "Active Message Format over UDP.\n\n"
            "Run outside the container (requires Python 3.8+ for j2735):\n"
            "  1) Install deps from this folder: ./install_dependencies.sh\n"
            "  2) Provide a config JSON with intersections, controller IPs, and signal_groups.\n\n"
            "Adapter Configuration:\n"
            "  - To publish encoded J2735 hex over TENA V2X Messages: \n"
            "    - Set VUG_DOCKER_START_V2X_ADAPTER=true in your site config \n"
            "    - Match IP/port with adapter: VUG_V2X_ADAPTER_RECEIVE_ADDRESS should equal --ip and \n"
            "      VUG_V2X_ADAPTER_RECEIVE_PORT should equal --port - you can pass them directly: \n"
            "        --ip $VUG_V2X_ADAPTER_RECEIVE_ADDRESS --port $VUG_V2X_ADAPTER_RECEIVE_PORT\n\n"
            "  - To convert the TENA V2X Messages into Traffic Signal Controller data for visualization: \n"
            "    - Set VUG_DOCKER_START_ENTITY_GENERATOR=true \n"
            "    - Ensure every intersection ID you send exists in the scenario XML under \n"
            "      intersectionSignalControllers and phaseSignalMappings\n\n"
            "Examples:\n"
            "  - Config-driven intersections to a local receiver:\n"
            "      ./ntcip_spat_generator.py --config ./ntcip_spat_config.json --ip 127.0.0.1 --port 1516\n\n"
            "  - Multiple intersections with slower rate (IDs/signal_groups come from config):\n"
            "      ./ntcip_spat_generator.py --config ./ntcip_spat_config.json --hz 5\n\n"
            "Press Ctrl+C to stop the generator."
        )
    )

    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="Print raw SPaT JER payloads before encoding",
    )

    parser.add_argument(
        "--config",
        type=str,
        required=True,
        help=(
            "Path to JSON config with per-intersection signal_groups. "
            "Intersections emitted are derived from the config; no default intersection IDs."
        ),
    )

    parser.add_argument(
        "--hz",
        type=float,
        default=10.0,
        help="Output rate in Hz for all intersections (default: 10.0)",
    )

    # UDP target
    parser.add_argument(
        "--ip",
        type=str,
        default="127.0.0.1",
        help="Destination IP for Active Message Format UDP (default: 127.0.0.1)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=56700,
        help="Destination UDP port for Active Message Format (default: 56700)",
    )

    args = parser.parse_args()


    interval = 1.0 / args.hz

    intersections = load_phase_config(args.config)

    # UDP socket
    sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (args.ip, args.port)

    print("Press Ctrl+C to stop\n")
    print(
        f"Intersections: {[i['id'] for i in intersections]} | "
        f"Rate: {args.hz} Hz | "
        f"UDP target: {args.ip}:{args.port}\n"
    )

    try:
        while True:
            loop_start = time.time()

            # Compute controller state ONCE per tick
            moy, time_mark = compute_moy_and_time_mark()

            debug_info = {
                "timestamp": datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
            }

            # Build, encode, log, and send for each intersection
            for intersection in intersections:
                intersection_id = intersection["id"]
                signal_groups = intersection["signal_groups"]
                intersection_ip = intersection.get("ip")
                debug_info["signal_groups"] = signal_groups

                spat_jer = build_spat_for_intersection(
                    intersection_id,
                    intersection_ip,
                    moy,
                    time_mark,
                    signal_groups,
                )
                if args.verbose:
                    print(f"spat_jer: {spat_jer}")
                hex_str = encode_spat_to_uper_hex(spat_jer)

                # Human-readable log
                print_frame_log(intersection_id, debug_info, hex_str)

                # Build AMF text and send via UDP
                amf_text = build_active_message(hex_str)
                sk.sendto(amf_text.encode("ascii"), target)

            # Rate control
            sleep_time = interval - (time.time() - loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nStopped SPaT generator.")


if __name__ == "__main__":
    main()
