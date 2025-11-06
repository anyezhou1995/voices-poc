import argparse
import time
from scapy.all import IP, UDP, Raw, PcapWriter

# EXAMPLE SCRIPT EXECUTION: python3 convert_json_to_pcap.py ~/distributed-testing/scripts/utils/example_scenario.json -o scenario.pcap

def json_to_pcap(json_file, outfile, src_ip, dst_ip, src_port, dst_port):
    """Convert JSON text file into a .pcap with one UDP packet containing it."""
    with open(json_file, "r") as f:
        json_payload = f.read().strip()

    pkt = IP(src=src_ip, dst=dst_ip) / UDP(sport=src_port, dport=dst_port) / Raw(load=json_payload.encode())
    pkt.time = time.time()

    pw = PcapWriter(outfile, sync=True)
    pw.write(pkt)
    pw.close()

    print(f"Wrote {outfile} with one UDP packet containing JSON from {json_file}")

def main():
    ap = argparse.ArgumentParser(description="Package a JSON message as a PCAP for Replay via tcp_replay.py")
    ap.add_argument("json_file", help="Path to JSON file")
    ap.add_argument(
        "-o", "--outfile", default="scenario.pcap",
        help="Output pcap file name (default: scenario.pcap)"
    )
    ap.add_argument("--src-ip", default="127.0.0.1", help="Source IP")
    ap.add_argument("--dst-ip", default="127.0.0.1", help="Destination IP")
    ap.add_argument("--src-port", type=int, default=40000, help="Source UDP port")
    ap.add_argument("--dst-port", type=int, default=8005, help="Destination UDP port")

    args = ap.parse_args()

    json_to_pcap(
        json_file=args.json_file,
        outfile=args.outfile,
        src_ip=args.src_ip,
        dst_ip=args.dst_ip,
        src_port=args.src_port,
        dst_port=args.dst_port
    )

if __name__ == "__main__":
    main()

    