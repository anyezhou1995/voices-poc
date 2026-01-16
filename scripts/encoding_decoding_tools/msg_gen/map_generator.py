#!/usr/bin/env python3
import time
import socket
import argparse
from argparse import RawTextHelpFormatter

# ---------------------------------------------------------------------------
# Static configuration:
# ---------------------------------------------------------------------------

MAP_CONFIG = [
    {"intersection_id": 1, "payload": "0012811338063000200002194d4bd7223e227a25104602dc06580428000004000218f889d2466a5ebc318f889cc666a5ebcb18f889c4f66a5ebf418f889bb026a5ec5398f889b24e6a5eca918f889aa726a5ed0082c120000804b0045000000a000431f113a48cd4bd63c31f11399a4d4bd64c31f1138984d4bd6a331f113752cd4bd76231f11363ccd4bd80731f1135154d4bd8bc0581c0001008802110000000a63e227e489a97affa63e227fc69a97b00e63e22819c9a97b02463e22835d9a97b03863e2285769a97b04263e2287b69a97b05663e228a5c9a97b062200644000000218f889f9726a5eb5998f88a039e6a5eb6698f88a0c1a6a5eb6918f88a15026a5eb7098f88a1fce6a5eb7898f88a293a6a5eb7b0"},
    {"intersection_id": 2, "payload": "0012810738043000200004114d4bd8df3e22bbfa102802dc06580428000006000218f88add626a5ec5718f88ad1f66a5ec5718f88ac75e6a5ec5498f88abba26a5ec5218f88aae566a5ec4d38f88a9c6a6a5ec47814140b0480002012c011400000280010c7c456ea9352f5dc0c7c4568f3352f5dacc7c4563b7352f5d98c7c455dcb352f5d6cc7c455717352f5d59c7c454e2f352f5d1c0a0a0581c0001008801910000000863e22c1969a97b0c663e22c3799a97b10e63e22c66f9a97b18063e22c8ec9a97b1d263e22cb4e9a97b22463e22cf069a97b2a0200844000000198f88b05d66a5ecd698f88b0f7a6a5ecf098f88b1cc66a5ed1498f88b2e966a5ed4598f88b3d926a5ed6700"},
    {"intersection_id": 3, "payload": "0012810b38043020200006114d4bdd193e22ea28102802dc06580208000004000218f88b9f866a5edbf18f88b970a6a5edad18f88b90da6a5eda018f88b88426a5ed8b98f88b7d926a5ed7498f88b720e6a5ed5a82c160000804b00810000008000431f1173cf4d4bdcb431f1172d04d4bdc9031f1171f64d4bdc7631f1170ea4d4bdc5231f116f97cd4bdc1e31f116e26cd4bdbeb058340001008802810000000863e22ed7c9a97b7d663e22f0069a97b83263e22f2919a97b88463e22f4669a97b8ce63e22f6fe9a97b92a63e22f8e89a97b972200c04000000218f88bb56a6a5ee9018f88bbe6e6a5eea518f88bc93a6a5eebe98f88bd00a6a5eece18f88bdb3e6a5eee818f88be25e6a5eef780"},
    {"intersection_id": 4, "payload": "001280f038013000200008054d4be6463e235d81000002dc06580228000005000198f88d65266a5f26818f88d5d326a5f25b18f88d52326a5f24418f88d49ce6a5f23198f88d3eb26a5f21582c0e0000804b0085000000c000431f11ac974d4be61f31f11ab8ecd4be60031f11aa7bcd4be5d731f11a92fcd4be5a331f11a7c94d4be57031f11a6c34d4be5460482400010801910000000663e23624d9a97cb7863e2364f99a97cbf263e2366f09a97cc5063e2368ee9a97ccac63e236b439a97cd0a200844000000198f88d887a6a5f37d98f88d94026a5f39f98f88d9ece6a5f3be18f88da79a6a5f3d818f88dae9e6a5f3ea0"},
    {"intersection_id": 5, "payload": "001280e53802300020000a084d4bec343e23b20202dc065802280000050001b8f88eb68a6a5f59d0141463e23ac8f9a97d64a63e23aae29a97d60263e23a8de9a97d5ba63e23a62b9a97d5520b0380002012c021400000300009c7c475b1d352fb1940a0a31f11d5b0cd4bec3731f11d3d4cd4bebe931f11d2494d4beba60582400010088019100000006e3e23b6b99a97d704050518f88ee2726a5f5d318f88eea166a5f5e518f88ef02a6a5f5f498f88ef7ea6a5f6040802110000000663e23b6979a97d98e63e23b8379a97d9cc63e23ba279a97da1463e23bb9f9a97da5263e23bd9c9a97da860"},
    {"intersection_id": 6, "payload": "001281493805300020000c154d4bef4a3e2404b20ff602dc08500648000002000198f88ff2126a5f82598f88fe7de6a5f84b98f88fda526a5f88098f88fcdee6a5f89018f88fbe226a5f88096010a0000010000863e23fefd9a97dbdc63e23fcce9a97dcf263e23f9639a97de8a63e23f5bb9a97e00c63e23f2df9a97e0c863e23efe09a97e13c0b0580002012c011400000280010c7c47fcd3352fb158c7c47f6c1352fb500c7c47f115352fb884c7c47ead5352fbb2cc7c47e51d352fbca0c7c47dfa7352fbd0016090000402300864000004000218f890320a6a5f51e98f8903c866a5f4bc98f89049ee6a5f45818f890546a6a5f41018f89060ea6a5f3d698f89071066a5f3a98c02990000010000a63e240d089a97d6a863e240f3c9a97d59063e2411a59a97d47063e24145e9a97d34e63e2416f69a97d27663e241a3e9a97d17863e241c419a97d1420"},
    {"intersection_id": 7, "payload": "0012811e3802302020000e084d4be9fd3e243baa02dc06580228000004000298f890e62a6a5f45d18f890dbe66a5f43d98f890d16e6a5f42698f890c66e6a5f40c98f890b9a66a5f3eb18f890b0026a5f3d398f890a8926a5f3c402c0e0000804b00850000008000531f121caacd4bea0431f121b72cd4be9d531f121a194d4be9a631f1218c34d4be97331f121712cd4be93f31f1215e54d4be91631f12150b4d4be8f7058240001008801910000000863e243e419a97d27e63e2440899a97d2c463e2443cf9a97d340e3e2446749a97d392050518f89123926a5f4fc18f8912f866a5f51d8802110000000863e243e209a97d5a463e24403f9a97d5f663e2442b69a97d63ce3e2445769a97d698050518f891210e6a5f5c018f8912e426a5f5df0"},
    {"intersection_id": 8, "payload": "0012811238053020200010154d4bbf403e248c7b0ff602dc06580228000005000298f89221966a5defc18f89218666a5dedc18f8920f466a5debd18f89204b26a5deb598f891fa6e6a5decf18f891f05e6a5df0a98f891e8666a5df5d02c0e0000804b0085000000c000531f12440d4d4bbf4131f124323cd4bbf0d31f1241d14d4bbec531f12407ecd4bbecf31f123f5a4d4bbf0031f123e444d4bbf5731f123d38cd4bbffa058240001008c01910000010000663e24913c9a977e4e63e2492e19a977e96e3e2495b59a977f0804fb18f89265ee6a5dfed98f89273da6a5e0198c02110000010000663e2490d19a97810a63e2492b99a97814a63e2495579a9781a6e3e2499529a97824004fb18f89272ea6a5e0b98"},
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