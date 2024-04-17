from scapy.all import PcapReader, PcapWriter, raw

def filter_packets_with_pattern(source_file, destination_file, pattern):
    # Open the source PCAP file
    packets = PcapReader(source_file)
    
    # Create a new PCAP file for the filtered packets
    writer = PcapWriter(destination_file, append=True, sync=True)

    # Iterate through packets in the PCAP file
    for packet in packets:
        # Convert the packet to its raw bytes form
        # packet_bytes = raw(packet)
        packet_bytes = packet.load
        print(f'packet_bytes: {packet_bytes}')

        # Check if the pattern is in the packet's payload
        # This example assumes you're looking for a byte pattern.
        # Convert your pattern to bytes if it's a string, e.g., b'yourpattern'
        if pattern.encode() in packet_bytes:
            # If the pattern does not match, write the packet to the new PCAP file
            print("\tKeeping packet")
            writer.write(packet)
    
    # Close the writer to ensure all packets are flushed to the file
    writer.close()

    print(f"Processed packets. Output saved to {destination_file}")

# Usage example
source_pcap = '/home/vug/Downloads/econ_spat_03272024.pcap'
destination_pcap = '/home/vug/Downloads/econ_spat_03272024-spat_only-test.pcap'
pattern = 'Payload=0013'

filter_packets_with_pattern(source_pcap, destination_pcap, pattern)