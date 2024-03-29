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

from binascii import unhexlify
# import J2735_201603_combined
import J2735_201603_combined_voices_mr_fix as J2735

decode_j3224 = True


try:
    import SDSMDecoder
    # import SDSM as SDSM
except Exception as errMsg:
    print("WARNING: Unable to import J3224 python library, skipping all J3224 messages")
    decode_j3224 = False


import json
import sys
import csv
import numpy
import datetime
import os
import shutil
import time

print("\n----- DECODING J2735 PACKETS -----")

# print("\nARG LIST:")
# for i, arg in enumerate(sys.argv):
#     print(str(i) + ": " + str(arg))
# print("")

inFile = sys.argv[1]
outfile = sys.argv[2]
destination_dir = sys.argv[3]

os.chdir(destination_dir)

def extract_values(obj, key):
    """Pull all values of specified key from nested JSON."""
    arr = []

    def extract(obj, arr, key):
        """Recursively search for values of key in JSON tree."""
        if isinstance(obj, dict):
            for k, v in obj.items():
                if isinstance(v, (dict, list)):
                    extract(v, arr, key)
                elif k == key:
                    arr.append(v)
        elif isinstance(obj, list):
            for item in obj:
                extract(item, arr, key)
        return arr

    results = extract(obj, arr, key)
    return results

def convID(id, length):
    id = id.hex()
    i = 0
    if (length == 8):
        while(i<21):
            id = id[:i+2] + " " + id[i+2:]
            i += 3
    else:
        while(i<45):
            id = id[:i+2] + " " + id[i+2:]
            i += 3

    id = list(id.split(" "))

    for x in range(len(id)):
        inted = int(id[x], 16)
        id[x] = inted

    return id


infile_obj = open(inFile,'r')

# decoded_output_folder = outfile.replace(".csv","")

# if os.path.exists(decoded_output_folder):
#     print("\nRemoving leftover files from previous failed run")
#     shutil.rmtree(decoded_output_folder)

# os.makedirs(decoded_output_folder)

numSpatPhases = 31 #use one more than desired phases

spatColumnHeaderList=["packetIndex","packetTimestamp","spatTimestamp","intersectionID","intersectionName"]
for headerPhase in range(1,numSpatPhases):
    spatColumnHeaderList.append("phase" + str(headerPhase) + "_eventState")

# NOTE: all types will have hex payload and JSON appended to end of list
message_types = {
    "BSM" : {
        "name" : "BSM",
        "message_spec" : "J2735",
        "message_id_hex" : "0014",
        "column_headers" : ["packetIndex","packetTimestamp","bsm id","secMark","latency","latitude","longitude","speed(m/s)","heading","elevation(m)","accel_long(m/s^2)"],
    },
    "SPAT" : {
        "name" : "SPAT",
        "message_spec" : "J2735",
        "message_id_hex" : "0013",
        "column_headers" : ["packetIndex","packetTimestamp","spatTimestamp","intersectionID","intersectionName"],
    },
    "MAP" : {
        "name" : "MAP",
        "message_spec" : "J2735",
        "message_id_hex" : "0012",
        "column_headers" : ["packetIndex","packetTimestamp","intersectionID"],
    },
    "Mobility-Request" : {
        "name" : "Mobility-Request",
        "message_spec" : "J2735",
        "message_id_hex" : "00f2",
        "column_headers" : ["packetIndex","packetTimestamp","headerTimestamp","hostStaticId","hostBSMId","planId","strategy","planType","urgency","strategyParams","trajectoryStart","trajectory","expiration"],
    },
    "Mobility-Response" : {
        "name" : "Mobility-Response",
        "message_spec" : "J2735",
        "message_id_hex" : "00f3",
        "column_headers" : ["packetIndex","packetTimestamp","headerTimestamp","hostStaticId","hostBSMId","planId","urgency","isAccepted"],
    },
    "Mobility-Path" : {
        "name" : "Mobility-Path",
        "message_spec" : "J2735",
        "message_id_hex" : "00f0",
        "column_headers" : ["packetIndex","packetTimestamp","hostStaticId","hostBSMId","planId","location","trajectory"],
    },
    "Mobility-Operations" : {
        "name" : "Mobility-Operations",
        "message_spec" : "J2735",
        "message_id_hex" : "00f1",
        "column_headers" : ["packetIndex","packetTimestamp","headerTimestamp","hostStaticId","hostBSMId","planId","strategy","operationParams"],
    },
    "Platooning" : {
        "name" : "Platooning",
        "message_spec" : "J2735",
        "message_id_hex" : "NA",
        "column_headers" : ["platooningPacketType","packetIndex","packetTimestamp","headerTimestamp","hostStaticId","hostBSMId","planId","strategy","other_params"],
    },
    "Traffic-Control-Request" : {
        "name" : "Traffic-Control-Request",
        "message_spec" : "J2735",
        "message_id_hex" : "00f4",
        "column_headers" : ["packetIndex","packetTimestamp","reqid_hex","reqid_dec_list","reqseq","scale","bounds"],
    },
    "Traffic-Control-Message" : {
        "name" : "Traffic-Control-Message",
        "message_spec" : "J2735",
        "message_id_hex" : "00f5",
        "column_headers" : ["packetIndex","packetTimestamp","reqid_hex","reqid_dec_list","reqseq","msgtot","msgnum","tcmID_hex","tcmID_dec_list","updated","label","tcID_hex","tcID_dec_list","vclasses","schedule","detail","geometry"],
    },
    "J2735-Payload" : {
        "name" : "J2735-Payload",
        "message_spec" : "J2735",
        "message_id_hex" : "NA",
        "column_headers" : ["packetIndex","packetTimestamp"],
    },
    "Sensor-Data-Sharing-Message" : {
        "name" : "Sensor-Data-Sharing-Message",
        "message_spec" : "J3224",
        "message_id_hex" : "0029",
        "column_headers" : ["packetIndex","packetTimestamp"],
    },
    "J3224-Payload" : {
        "name" : "J3224-Payload",
        "message_spec" : "J3224",
        "message_id_hex" : "NA",
        "column_headers" : ["packetIndex","packetTimestamp"],
    },
}

message_id_hex_lookup = {}

for message_type in message_types:
    message_id_hex_lookup[str(message_types[message_type]["message_id_hex"])] = message_type
    message_types[message_type]["outfile_name"] = outfile.replace(".csv","_" + message_types[message_type]["name"] + ".csv")
    message_types[message_type]["outfile_obj"] = open(message_types[message_type]["outfile_name"],'w',newline='')
    message_types[message_type]["outfile_writer"] = csv.writer(message_types[message_type]["outfile_obj"])
    message_types[message_type]["column_headers"] += ["hex_payload","JSON"]
    message_types[message_type]["outfile_writer"].writerow(message_types[message_type]["column_headers"])



infile_reader = csv.reader(infile_obj,delimiter=',')
packet_list = list(infile_reader)[1:]

latency_array = []
prevSpatTimestamp = 0

trimmed_packet_column = 5
message_type_id_column = 4

for packet in packet_list:

    message_type_obj = message_types[message_id_hex_lookup[packet[message_type_id_column]]]

    msg_row = []
    platoon_row = []
    j2735_payload_row = []
    j3224_payload_row = []

    if packet[message_type_id_column] == "0029":
        try:
            msg = SDSMDecoder.sdsm_decoder(packet[trimmed_packet_column])
        except Exception as e:
            print("\n  DECODING ERROR: " + str(e))
            print("    [" + str(packet[0])+'] ' + str(packet[2]) )
            continue
        
        j3224_payload_row = [str(packet[0]),str(packet[1]),str(packet[trimmed_packet_column])]

    else:

        msg = J2735.DSRC.MessageFrame
        #print("hex: " + packet[trimmed_packet_column] + " byte length: " + str(len(packet[trimmed_packet_column])))
        try:
            msg.from_uper(unhexlify(packet[trimmed_packet_column]))
        except Exception as e:
            print("\n  DECODING ERROR: " + str(e))
            print("    [" + str(packet[0])+'] ' + str(packet[2]) )
            continue
        
        j2735_payload_row = [str(packet[0]),str(packet[1]),str(packet[trimmed_packet_column])]

    if (packet[message_type_id_column] == "0013") :
        # print("Parsing SPAT")
        try:
            spatPhaseArray = [""] * numSpatPhases
            intersectionID = msg()['value'][1]['intersections'][0]['id']['id']
            try:
                intersectionName = msg()['value'][1]['intersections'][0]['name']
            except:
                intersectionName = ""
            spatTimestamp = msg()['value'][1]['intersections'][0]['timeStamp']
            instersectionPhaseArray = msg()['value'][1]['intersections'][0]['states']
            
            for phase in range(len(instersectionPhaseArray)):
                currentPhase = msg()['value'][1]['intersections'][0]['states'][phase].get('signalGroup')
                currentState = str(msg()['value'][1]['intersections'][0]['states'][phase]['state-time-speed'][0]['eventState'])
                spatPhaseArray[currentPhase] = currentState
            
            msg_row = [str(packet[0]),str(packet[1]),str(spatTimestamp),str(intersectionID),intersectionName ]
            
            for printPhase in range(1,numSpatPhases):
                msg_row.append(spatPhaseArray[printPhase])
            msg_row.append(str(packet[trimmed_packet_column]))
            
        except:
            print("ERROR PARSING DECODED PACKET: " )
            print("[" + str(packet[0])+"] " + str(packet[5]) + " - " + str(msg()['value']))

            msg_row = [str(packet[0]),str(packet[1]),"","",""]
            
            for printPhase in range(1,numSpatPhases):
                msg_row.append("")

            msg_row.append(str(packet[trimmed_packet_column]))

    elif (packet[message_type_id_column] == "0012") :
        # print("Parsing MAP")
        try:
            intersectionID = msg()['value'][1]['intersections'][0]['id']['id']
            #if intersectionID == intersection:
            lat = msg()['value'][1]['intersections'][0]['refPoint']['lat']
            longstr = msg()['value'][1]['intersections'][0]['refPoint']['long']
            laneWidth = msg()['value'][1]['intersections'][0]['laneWidth']

            msg_row = [str(packet[0]),str(packet[1]),str(intersectionID),str(lat/10000000.0),str(longstr/10000000.0),str(laneWidth)]

        except:
            print("ERROR PARSING DECODED PACKET: " )
            print("[" + str(packet[0])+"] " + str(packet[5]) + " - " + str(msg()['value']))

            msg_row = [str(packet[0]),str(packet[1]),"","","",""]

    elif (packet[message_type_id_column] == "0014") : # if bsm , look for lat, long, speed along with time
        # print("Parsing BSM")

        try:
            bsmId = msg()['value'][1]['coreData']['id']
            lat= msg()['value'][1]['coreData']['lat']
            longstr = msg()['value'][1]['coreData']['long']
            speed = msg()['value'][1]['coreData']['speed']
            elevation = msg()['value'][1]['coreData']['elev']
            secMark = msg()['value'][1]['coreData']['secMark']
            heading = msg()['value'][1]['coreData']['heading']
            speed_converted = speed*0.02 #m/s
            accel_long = msg()['value'][1]['coreData']['accelSet']['long']
            accel_long_converted = accel_long*0.01 #m^s^2
            
            packet_timestamp = datetime.datetime.fromtimestamp(int(float(packet[0])))
            roundDownMinTime = datetime.datetime(packet_timestamp.year,packet_timestamp.month,packet_timestamp.day,packet_timestamp.hour,packet_timestamp.minute).timestamp()
            packetSecondsAfterMin = (float(packet[0]) - roundDownMinTime)
            latency = packetSecondsAfterMin*1000 - secMark
            if (latency < 0) :
                #print("[!!!] Minute mismatch")
                latency = latency + 60000
            #print("latency: " + str(latency))
            
            latency_array.append(latency)
            
            msg_row = [str(packet[0]),str(packet[1]),str(bsmId.hex()),str(secMark),str(latency),str(lat/10000000.0),str(longstr/10000000.0),str(speed_converted),str(heading),str(accel_long_converted), str(elevation)]

        except:
            print("ERROR PARSING DECODED PACKET: " )
            print("[" + str(packet[0])+"] " + str(packet[5]) + " - " + str(msg()['value']) )

            msg_row = [str(packet[0]),str(packet[1]),"","","","","","","","","",str(packet[trimmed_packet_column])]

    elif (packet[message_type_id_column] == "00f0") :
        # print("Parsing Mobility Request")

        hostStaticId = msg()['value'][1]['header']['hostStaticId']
        hostBSMId = msg()['value'][1]['header']['hostBSMId']
        planId = msg()['value'][1]['header']['planId']
        headerTimestamp = msg()['value'][1]['header']['timestamp']
        strategy = msg()['value'][1]['body']['strategy']
        planType = msg()['value'][1]['body']['planType']
        urgency = msg()['value'][1]['body']['urgency']
        strategyParams = str(msg()['value'][1]['body']['strategyParams'])
        trajectoryStart = str(msg()['value'][1]['body']['trajectoryStart'])
        trajectory = msg()['value'][1]['body']['trajectory']
        expiration = msg()['value'][1]['body']['expiration']

        msg_row = [str(packet[0]),str(packet[1]), str(headerTimestamp),str(hostStaticId),str(hostBSMId),str(planId),str(strategy),str(planType),str(urgency),str(strategyParams),str(trajectoryStart),str(trajectory),str(expiration)]

        platoon_row = ["Mobility_Request" , str(packet[0]),str(packet[1]),str(hostStaticId),str(hostBSMId),str(planId),str(strategy),str(planType),str(urgency),str(strategyParams),str(trajectoryStart),str(trajectory),str(expiration)]


    elif (packet[message_type_id_column] == "00f1") :
        # print("Parsing Mobility Response")

        hostStaticId = msg()['value'][1]['header']['hostStaticId']
        hostBSMId = msg()['value'][1]['header']['hostBSMId']
        planId = msg()['value'][1]['header']['planId']
        headerTimestamp = msg()['value'][1]['header']['timestamp']
        urgency = msg()['value'][1]['body']['urgency']
        isAccepted = msg()['value'][1]['body']['isAccepted']

        msg_row = [str(packet[0]),str(packet[1]), str(headerTimestamp),str(hostStaticId),str(hostBSMId),str(planId),str(urgency),str(isAccepted)]
        platoon_row = ["Mobility_Response",str(packet[0]),str(packet[1]),str(hostStaticId),str(hostBSMId),str(planId),str(urgency),str(isAccepted)]

    elif (packet[message_type_id_column] == "00f2") :
        # print("Parsing Mobility Path")

        hostStaticId = msg()['value'][1]['header']['hostStaticId']
        hostBSMId = msg()['value'][1]['header']['hostBSMId']
        planId = msg()['value'][1]['header']['planId']
        location = msg()['value'][1]['body']['location']
        trajectory = msg()['value'][1]['body']['trajectory']

        msg_row = [str(packet[0]),str(packet[1]),str(hostStaticId),str(hostBSMId),str(planId),str(location),str(trajectory)]

    elif (packet[message_type_id_column] == "00f3") :
        # print("Parsing Mobility Operations")

        hostStaticId = msg()['value'][1]['header']['hostStaticId']
        hostBSMId = msg()['value'][1]['header']['hostBSMId']
        planId = msg()['value'][1]['header']['planId']
        headerTimestamp = msg()['value'][1]['header']['timestamp']
        strategy = msg()['value'][1]['body']['strategy']
        operationParams = msg()['value'][1]['body']['operationParams']
        
        msg_row = [str(packet[0]),str(packet[1]), str(headerTimestamp) ,str(hostStaticId),str(hostBSMId),str(planId),str(strategy),str(operationParams)]
        platoon_row = ["Mobility_Operations",str(packet[0]),str(packet[1]),str(hostStaticId),str(hostBSMId),str(planId),str(strategy),str(operationParams)]

    elif (packet[message_type_id_column] == "00f4") :
        # print("Parsing Mobility TCR")

        reqid = msg()['value'][1]['body'][1]['reqid']
        reqseq = msg()['value'][1]['body'][1]['reqseq']
        scale = msg()['value'][1]['body'][1]['scale']
        bounds = msg()['value'][1]['body'][1]['bounds']

        newReqId = str(convID(reqid, 8)).replace(",", " ")
        reqid = reqid.hex()
        
        msg_row = [str(packet[0]),str(packet[1]),reqid,newReqId,str(reqseq),str(scale),str(bounds)]
    
    elif (packet[message_type_id_column] == "00f5") :
        # print("Parsing TCM")
        
        reqid = msg()['value'][1]['body'][1]['reqid']
        reqseq = msg()['value'][1]['body'][1]['reqseq']
        msgtot = msg()['value'][1]['body'][1]['msgtot']
        msgnum = msg()['value'][1]['body'][1]['msgnum']
        tcmId = msg()['value'][1]['body'][1]['id']
        updated = msg()['value'][1]['body'][1]['updated']
        label = msg()['value'][1]['body'][1]['package']['label']
        tcId = msg()['value'][1]['body'][1]['package']['tcids'][0]
        vclasses = msg()['value'][1]['body'][1]['params']['vclasses']
        schedule = msg()['value'][1]['body'][1]['params']['schedule']
        detail = msg()['value'][1]['body'][1]['params']['detail']
        geometry = msg()['value'][1]['body'][1]['geometry']

        newReqId = str(convID(reqid, 8)).replace(",", " ")
        newTcmId = str(convID(tcmId, 16)).replace(",", " ")
        newtcId = str(convID(tcId, 16)).replace(",", " ")
        
        reqid = reqid.hex()
        tcmId = tcmId.hex()
        tcId = tcId.hex()
        
        msg_row = [str(packet[0]),str(packet[1]),reqid,newReqId,str(reqseq),str(msgtot),str(msgnum),tcmId,newTcmId,str(updated),str(label),tcId,newtcId,str(vclasses),str(schedule),str(detail),str(geometry),]

    elif (packet[message_type_id_column] == "0029") :
        # print("Parsing SDSM")
        # print("Msg: " + str( msg))

        msg_row = [str(packet[0]),str(packet[1])]

    else:
        print("\nERROR: NO MATCHING MESSAGE TYPES FOR PAYLOAD: ")
        print("[" + str(packet[0])+'] ' + str(packet[2]) )


    # append hex payload and JSON
    msg_row.append(str(packet[trimmed_packet_column]))
    try:
        msg_json = msg()
    except:
        msg_json = msg

    msg_row.append(msg_json)

    # print("msg_row: " + str(msg_row))
    # write row
    if msg_row:
        message_type_obj["outfile_writer"].writerow(msg_row)

    if platoon_row:
        message_types["Platooning"]["outfile_writer"].writerow(msg_row)

    if j2735_payload_row:
        message_types["J2735-Payload"]["outfile_writer"].writerow(msg_row)
    
    if j3224_payload_row:
        message_types["J3224-Payload"]["outfile_writer"].writerow(msg_row)




print("")

for message_type in message_types:
    message_types[message_type]["outfile_obj"].close()
    message_types[message_type]["outfile_obj_read"] = open(message_types[message_type]["outfile_name"])
    message_types[message_type]["num_outfile_rows"] = sum(1 for line in message_types[message_type]["outfile_obj_read"])
    num_messages = int(message_types[message_type]["num_outfile_rows"] -1)
    print(message_types[message_type]["name"] + ": " + str(num_messages))
    message_types[message_type]["outfile_obj"].close()

    # if there are no messages we dont need to keep the empty file
    if num_messages == 0:
        os.remove(message_types[message_type]["outfile_name"])

# if (payload_type_id == "0014") : 
#     print("")
#     print("---------- Performance Metrics ----------")
#     latency_avg = numpy.average(latency_array)
#     print("Latency Average: " + str(latency_avg))
#     latency_std = numpy.std(latency_array)
#     print("Latency Standard Deviation (Jitter): " + str(latency_std))

