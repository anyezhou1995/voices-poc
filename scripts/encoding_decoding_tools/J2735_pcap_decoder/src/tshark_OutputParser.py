#!/usr/bin/env python3

#  *                                                                              
#  * Copyright (C) 2022 LEIDOS.                                              
#  *                                                                              
#  * Licensed under the Apache License, Version 2.0 (the "License"); you may not  
#  * use this file except in compliance with the License. You may obtain a copy o\
# f                                                                               
#  * the License at                                                               
#  *                                                                              
#  * http://www.apache.org/licenses/LICENSE-2.0                                   
#  *                                                                              
#  * Unless required by applicable law or agreed to in writing, software          
#  * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT    
#  * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the     
#  * License for the specific language governing permissions and limitations under                                                                               
#  * the License.                                                                 
#  *


import json
import sys
from csv import reader, writer
import re

print("\n----- CHOPPING HEADERS AND IDENTIFYING PACKETS -----")

# print("\nARG LIST:")
# for i, arg in enumerate(sys.argv):
#     print(str(i) + ": " + str(arg))
# print("")

inFile = sys.argv[1]
outfile = sys.argv[2]
payload_type = sys.argv[3]

message_type_list = [
    {
        'name'  : 'MAP',
        'id'    : '0012'
    },
    {
        'name'  : 'SPAT',
        'id'    : '0013'
    },
    {
        'name'  : 'BSM',
        'id'    : '0014'
    },
    {
        'name'  : 'Mobility Request',
        'id'    : '00f0'
    },
    {
        'name'  : 'Mobility Response',
        'id'    : '00f1'
    },
    {
        'name'  : 'Mobility Path',
        'id'    : '00f2'
    },
    {
        'name'  : 'Mobility Operations',
        'id'    : '00f3'
    },
    {
        'name'  : 'Traffic Control Request',
        'id'    : '00f4'
    },
    {
        'name'  : 'Traffic Control Message',
        'id'    : '00f5'
    },
    {
        'name'  : 'Sensor Data Sharing Message',
        'id'    : '0029'
    }
]

with open(inFile) as read_obj, \
open(outfile, 'w', newline='') as write_obj:
    csv_reader = reader(read_obj)
    csv_writer = writer(write_obj)

    found_packets = 0

    # write the headers for the output csv
    if payload_type == "hex":
        csv_writer.writerow(["packet_index","timestamp","frame_raw","message_type","message_type_id","cleaned_payload"])
    elif payload_type == "ascii":
        csv_writer.writerow(["packet_index","timestamp","payload","message_type","message_type_id","cleaned_payload"])
    else:
        print("\nERROR: Invalid payload type")
        exit

    # skip first row for headers
    next(csv_reader)

    fixed_header_mode = False

    for row in csv_reader:
        
        packet_index = row[0]
        timestamp = row[1]
        current_packet_data = row[2]

        # print("packet_index: " + str(packet_index))
        # print("timestamp: " + str(timestamp))
        # print("current_packet_data: " + str(current_packet_data))
        
        if payload_type == "hex":
        # look for all message types in the raw packet frame (HEX) or payload (ascii)
        # identify where the pattern is found and choose the pattern that is closest to the beginning of the packet 
        # For hex, we know that all HEX J2735 payloads start with 0380 and varying number of hex values before the message type id
            
            
            if not fixed_header_mode:
                # this is the message type id which appears closest to the header
                # this starts at an arbitrary value
                closest_message_type = {
                    "name"  : "NA",
                    "id"    : "0000",
                    "pos"   : 99999   
                }

                num_matching_message_types = 0
                
                for message_type in message_type_list:
                    
                    j2735_payload_with_header_search = re.search('0380.*'+ message_type['id'],current_packet_data)

                    # print("Search: " + str(j2735_payload_with_header_search))
                    
                    if j2735_payload_with_header_search != None:
                        # print("Found: " + message_type['name'])
                        num_matching_message_types += 1

                        j2735_payload_with_header = str(j2735_payload_with_header_search.group(0))
                        search_result_start = j2735_payload_with_header_search.start()
                        search_result_end = j2735_payload_with_header_search.end()
                        # print("Start: " + str(search_result_start))
                        # print("End: " + str(search_result_end))
                        # print("With Header: " + str(j2735_payload_with_header))
                        
                        if search_result_end < closest_message_type["pos"]:
                            closest_message_type["name"] = message_type['name']
                            closest_message_type["id"] = message_type['id']
                            closest_message_type["pos"] = search_result_end
                # print("num_matching_message_types: " + str(num_matching_message_types))
                    
                # print("Closest Message Type: " + closest_message_type["name"])
                final_message_type_name = closest_message_type["name"]
                final_message_type_id = closest_message_type["id"]
                

                if closest_message_type['name'] == "NA":

                    print("\n --> No matching message types for payload, switching to fixed header mode")
                    # print("[" + str(packet_index)+'] ' + str(current_packet_data))
                    fixed_header_mode = True
                else:
                    # now that we know which message type we want, we want to grab the entire packet not just the pattern
                    final_j2735_payload_with_header_search = re.search('0380.*'+ closest_message_type['id'] + '.*',current_packet_data)
                    final_j2735_payload_with_header = str(final_j2735_payload_with_header_search.group(0))
                    
                    # trim off the 0380*** header
                    final_j2735_payload_with_header_cleaned_search = re.search(closest_message_type['id'] + '.*',final_j2735_payload_with_header)
                    final_j2735_payload_cleaned = str(final_j2735_payload_with_header_cleaned_search.group(0))
                            
                    
                    

            if fixed_header_mode:
                final_message_type_id = current_packet_data[84:88]
                final_message_type_obj = next((x for x in message_type_list if x["id"] == final_message_type_id), None)
                
                if final_message_type_obj == None:
                    print("\nERROR: INVALID PAYLOAD TYPE FOR FIXED HEADER MODE. UNABLE TO IDENTIFY PACKET")
                    print("[" + str(packet_index)+'] ' + str(current_packet_data))
                    continue

                final_message_type_name = final_message_type_obj["name"]
                final_j2735_payload_cleaned = current_packet_data[84:]

            # print("\nfinal_message_type_id: " + final_message_type_id)
            # print("final_message_type_name: " + final_message_type_name)
            # # print("timestamp: " + timestamp)
            # print("final_j2735_payload_cleaned: " + final_j2735_payload_cleaned)

            

            csv_writer.writerow([packet_index,timestamp,current_packet_data,final_message_type_name,final_message_type_id,final_j2735_payload_cleaned])

            # print("")
        elif payload_type == "ascii":
            
            current_payload_id = current_packet_data[0:4]
            
            current_packet_message_type = {
                    'name'  : 'NA',
                    'id'    : '0000'
                }

            for message_type in message_type_list:
                if message_type['id'] == current_payload_id:
                    current_packet_message_type = message_type

            if current_packet_message_type['name'] == "NA":
                print("\nERROR: NO MATCHING MESSAGE TYPES FOR PAYLOAD: ")
                print("[" + str(packet_index)+'] ' + str(current_packet_data))

            csv_writer.writerow([packet_index,timestamp,current_packet_data,current_packet_message_type["name"],current_packet_message_type["id"],current_packet_data])
        
        else:
            print("\nERROR: NO MATCHING MESSAGE TYPES FOR PAYLOAD: ")
            print("[" + str(packet_index)+'] ' + str(current_packet_data) )

# Old for Kapsch payloads. Testing with Kapsch payloads needed to include in updated 

        # if "Kapsch" in fileName[0] and bsm_substr in row[2]:
        #     bsm_index = row[2].find(bsm_substr)
        #     bsm_length = len(row[2])

        # elif spat_index < map_index and spat_index < bsm_index and "mk5" in fileName[0] and "rx" in fileName[0]:
        #     spat = row[1][spat_index:spat_length-8]
        #     csv_writer.writerow([row[0], spat])
        # if bsm_index < map_index and bsm_index < spat_index and "CohdaRSU_rx" in fileName[0]:
        #     bsm = row[1][bsm_index:bsm_length-8]
        #     csv_writer.writerow([row[0], bsm])
        # elif bsm_index < map_index and bsm_index < spat_index and "Kapsch" in fileName[0]:
        #     bsm = row[2][bsm_index:]
        #     csv_writer.writerow([row[0], bsm])
