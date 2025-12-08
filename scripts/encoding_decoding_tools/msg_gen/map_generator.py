#!/usr/bin/env python3
import time
import socket
import argparse
from argparse import RawTextHelpFormatter

# ---------------------------------------------------------------------------
# Static configuration:
# ---------------------------------------------------------------------------

MAP_CONFIG = [
    {"intersection_id": 1, "payload": "0012819538013020306052054d4bd83b3e2278af104602dc0514f8316010a0000014000a63e2272b09a97af1a63e226e439a97b01663e2268c99a97b2f663e225af69a97bbaae3e224be89a97c4cc04fb18f88911866a5f23798f888e2126a5f4060a4160000920640004580228000004000318f889caee6a5ec5a98f889b9c66a5ecab98f889a3566a5ed7198f889706a6a5ef6d38f88932ba6a5f1cb813ec63e2245629a97cb8863e223efe9a97cefa63e2239bf9a97d49809068000218032200000200000c7c44f025352f50b8c7c44f155352f1d54b010d0000002000331f113d274d4bd42e31f113d504d4bc74331f113d044d4bc22c31f113b7ccd4bbcfa71f11356bcd4ba856027d8120a40006300c84000004000018f889f8326a5ec9998f88a4e266a5ecbd8c02a10000010000063e227e179a97afaa63e2293899a97b0a6580ea8000002000318f889e0a66a5edb038f889ec566a5f0dd813ec63e227dc69a97ca5663e22816c9a97d4ba63e2281ef9a97de0a63e22819d9a97ee7ce3e22819d9a97fc08050518f889f7d26a6045d82419000100"},
    {"intersection_id": 2, "payload": "001281d938013020306030054d4bd8533e22bbe4102802dc0514f8496010a0000014000663e22b6789a97b18ce3e22aa249a97b154050538f88a7af26a5ec3c81414e3e228c459a97b06c050518f889d7326a5ebdb8a41a0000920640004580228000006000198f88ad9be6a5ecfa18f88aa8a66a5ecfc38f88a7b0a6a5ecdf01428e3e228c3c9a97b2c8050518f889d73a6a5ec710a41e0000921480004300644000004000018f88aea826a5e99c18f88aec4e6a5e2ae16021a0000010000663e22bce99a97a6be63e22bd279a978b62e3e22bd7c9a9780a804fb18f88af7e66a5dba0b8f88af9fe6a5d08e814142908800044831000216029a0000004000263e22be6a9a97a6b663e22be8e9a978b14e3e22bebb9a9781d204fb0241880010601908000008000031f1161eecd4bd9b031f116957cd4bdaf918074200000200000c7c45871b352f6bacc7c45a53b352f7048602148000008000031f115e9e4d4bdcbe71f115e7acd4beb1802828b02990000008000431f115cec4d4bdca631f115cc84d4be77671f115ce7cd4beae402828c7c4574df352fbe4dc7c4574c3352ff1080a0a71f115d024d4c09ee028281207000105812c8000002000098f88aecaa6a5ee5698f88aeb8e6a5f3d338f88aec1e6a5f56b01414090740008"},
]


def build_active_message(hex_payload, msg_type="MAP", psid="0x8002"):
    return (
        "Version=0.7\n"
        f"Type={msg_type}\n"
        f"PSID={psid}\n"
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


def main():
    parser = argparse.ArgumentParser(
        formatter_class=RawTextHelpFormatter,
        description=(
            "Broadcast static J2735 MAP payloads as AMF over UDP.\n"
            "Each intersection is staggered evenly across the period.\n"
        ),
    )

    parser.add_argument(
        "--period",
        type=float,
        default=1.0,
        help="Total cycle period in seconds (default: 1.0).",
    )

    parser.add_argument(
        "--ip",
        type=str,
        default="127.0.0.1",
        help="Destination IP for AMF UDP (default: 127.0.0.1)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=56702,
        help="Destination UDP port for AMF (default: 56702)",
    )

    parser.add_argument(
        "--type",
        type=str,
        default="MAP",
        help="AMF Type field (default: MAP)",
    )
    parser.add_argument(
        "--psid",
        type=str,
        default="0x8002",
        help="AMF PSID field (default: 0x8002 for MAP)",
    )

    parser.add_argument(
        "-q",
        "--quiet",
        action="store_true",
        help="Suppress per-send log lines",
    )

    args = parser.parse_args()

    if not MAP_CONFIG:
        print("MAP_CONFIG is empty – no intersections to send.")
        return

    num_intersections = len(MAP_CONFIG)
    period = args.period
    spacing = period / num_intersections

    sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (args.ip, args.port)

    print("Press Ctrl+C to stop\n")
    print(
        f"Configured intersections: {[c['intersection_id'] for c in MAP_CONFIG]}\n"
        f"Cycle period: {period:.3f}s | Spacing: {spacing:.3f}s\n"
        f"UDP target: {args.ip}:{args.port}\n"
    )

    try:
        while True:
            cycle_start = time.time()

            for idx, cfg in enumerate(MAP_CONFIG):
                send_start = time.time()

                amf_text = build_active_message(
                    hex_payload=cfg["payload"],
                    msg_type=args.type,
                    psid=args.psid,
                )
                sk.sendto(amf_text.encode("ascii"), target)

                if not args.quiet:
                    print(
                        f"[MAP-AMF] Sent intersection {cfg['intersection_id']} "
                        f"to {args.ip}:{args.port} | payload len={len(cfg['payload'])}"
                    )

                elapsed = time.time() - send_start
                wait = spacing - elapsed
                if idx != num_intersections - 1 and wait > 0:
                    time.sleep(wait)

            # keep cycle aligned to the period
            total_elapsed = time.time() - cycle_start
            remaining = period - total_elapsed
            if remaining > 0:
                time.sleep(remaining)

    except KeyboardInterrupt:
        print("\nStopped MAP AMF broadcaster.")


if __name__ == "__main__":
    main()
