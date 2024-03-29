#!/bin/bash

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


#	TODO
# - take files as input params
#
#
#
#

#exit on error
#
set -e
# set -x

function print_help {
	echo
	echo "usage: J2735_pcap_decoder.sh [-f <pcap file>]"
	echo
	echo "Decode J2735 pcap data"
	echo
	echo "optional arguments:"
	echo "    -f <pcap file> , --file <pcap file>   	input pcap file to be decoded"
	echo "    --help             						show help"
	echo
}



#this takes the location of the executed script as opposed to the current location
#this allows it to be run anywhere
executed_directory="`dirname \"$0\"`"
directory="`( cd \"$executed_directory\" && cd ../ && pwd )`"

cd $directory

no_tick_enabled=false
timeSyncEnabled=false
low_quality_flag=""
next_flag_is_input_file=false
pcap_file_to_read_arg=""
decoded_files_dest_dir_arg=""
keep_old_files_arg=false


for arg in "$@"
do
	if [ "$next_flag_is_input_file" = true ]; then

		pcap_file_to_read_arg=$arg
		next_flag_is_input_file=false
	
	elif [ "$next_flag_is_output_file" = true ]; then

		decoded_files_dest_dir_arg=$arg
		next_flag_is_output_file=false

	elif [[ $arg == "-i" ]] || [[ $arg == "--infile" ]]; then
		
		next_flag_is_input_file=true

	elif [[ $arg == "-o" ]] || [[ $arg == "--outfile" ]]; then
		
		next_flag_is_output_file=true

	elif [[ $arg == "--keep-old-files" ]]; then
		
		keep_old_files_arg=true
		
	elif [[ $arg == "-h" ]] || [[ $arg == "--help" ]]; then
		
		print_help
		exit

	elif [[ $arg != "" ]]; then
		
		echo
		echo "Invalid argument: $arg"
		print_help
		exit

	fi
done

# Ensure tshark is installed
REQUIRED_PKG="tshark"
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' $REQUIRED_PKG|grep "install ok installed")
echo Checking for $REQUIRED_PKG: $PKG_OK
if [ "" = "$PKG_OK" ]; then
  echo "No $REQUIRED_PKG found. Installing $REQUIRED_PKG."
  sudo apt-get update
  sudo apt-get --yes install $REQUIRED_PKG
fi

if [ -d data ]; then
	printf "\nFiles found in data directory:"
else
	mkdir data
fi

cd data

mkdir -p tsharkOutput
mkdir -p payloadOutput
mkdir -p decodedOutput

cd $directory

payloadType="hex"
messageTypeIdFound=false

extractPackets(){ 

#echo $input_pcap_base_name

#tshark -r $pcap_file_to_read --disable-protocol wsmp -Tfields -Eseparator=, -e frame.time_epoch -e data.data > $directory/data/tsharkOutput/$file_to_write

#searchForMessageTypeIdInFile $directory/data/tsharkOutput/$file_to_write $message_type_id 10

#search first 10 packets in text format and look for Payload=
#if this exist, string, we are ascii payloads and can decode that way
#if not, we must grab the payload from the frame_raw (tshark_json_parser)
search_for_ascii=$(tshark -r $pcap_file_to_read -o data.show_as_text:TRUE --disable-protocol wsmp -T fields -E separator=, -e data.text -c 10)
#search_for_ascii=$(tshark -r $pcap_file_to_read -o data.show_as_text:TRUE --disable-protocol wsmp -T fields -E separator=, -e frame.time_epoch -e data.text -c 10 | grep "Payload=")


search_for_ascii=${search_for_ascii//['\t\r\n']}

# echo "Search: " $search_for_ascii



# if [ true ]; then
if [[ $search_for_ascii = *[![:ascii:]]* || $search_for_ascii == "" ]]; then
        # The first packet contains the message id
		printf "\n --> PCAP is HEX\n"
		payloadType="hex"

		tshark_output_file=$input_pcap_base_name"_raw_packet_frames.csv"

		python3 $directory/src/tshark_json_parser.py $pcap_file_to_read $tshark_output_file
		mv $tshark_output_file $directory/data/tsharkOutput
        
else
        # The first packet does not contain the message id. This is most likely becuase payload is contained in ASCII
        printf "\n --> PCAP is in ascii\n"
		payloadType="ascii"

        # tshark -r $pcap_file_to_read -o data.show_as_text:TRUE --disable-protocol wsmp -T fields -E separator=, -e frame.time_epoch -e data.text > $directory/data/tsharkOutput/$raw_packet_output_file

		tshark_output_file=$input_pcap_base_name"_payloads.csv"

		python3 $directory/src/tshark_ascii_parser.py $pcap_file_to_read $tshark_output_file
		mv $tshark_output_file $directory/data/tsharkOutput

        #searchForMessageTypeIdInFile $directory/data/tsharkOutput/$raw_packet_output_file $message_type_id 10
                
fi

}


getPayload(){
	cd $directory/data/tsharkOutput
	parsed_tshark_file=$input_pcap_base_name"_filtered_packets.csv"
	#parse tshark output to get rid of unnecessary bytes in front of payloads
	python3 $directory/src/tshark_OutputParser.py $tshark_output_file $parsed_tshark_file $payloadType
	mv $parsed_tshark_file $directory/data/payloadOutput
}


decodePackets(){

	if [[ $decoded_files_dest_dir_arg == "" ]]; then

		output_dest=$directory/data/decodedOutput/$input_pcap_base_name"_decoded_packets"
	else
		output_dest=$decoded_files_dest_dir_arg
	fi

	cd $directory/data/payloadOutput

	if [ -d $output_dest ]; then
		if [[ $keep_old_files_arg != true ]]; then
			printf "\nResult files from previous decoding found, removing them"
			rm -rf $output_dest
			mkdir $output_dest
		fi

	else
		mkdir $output_dest
	fi

	decoded_file=$input_pcap_base_name"_decoded_packets.csv"
	python3 $directory/src/J2735decoder.py $directory/data/payloadOutput/$parsed_tshark_file $decoded_file $output_dest

	# decoded_output_file_pattern=$input_pcap_base_name"_decoded_packets"



	

	# mv $decoded_output_file_pattern $output_dest
	
}


generateKml(){ 
	echo 
	echo "Generating KML from CSV"
		
	mkdir -p $directory/data/kmlOutput

cat << EOF > $directory/data/kmlOutput/kmlOutput.kml
<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://earth.google.com/kml/2.2">
<Document>
  <name>BSMs</name>
  <description><![CDATA[KML of decoded BSMs]]></description>
  <Style id="end">
    <IconStyle>
      <Icon>
	<href>http://maps.gstatic.com/intl/en_us/mapfiles/ms/micons/red-dot.png</href>
      </Icon>
    </IconStyle>
  </Style>
  <Style id="middle">
    <IconStyle>
      <Icon>
	<href>http://maps.gstatic.com/intl/en_us/mapfiles/ms/micons/blue-dot.png</href>
      </Icon>
    </IconStyle>
  </Style>
  <Style id="start">
    <IconStyle>
      <Icon>
	<href>http://maps.gstatic.com/intl/en_us/mapfiles/ms/micons/green-dot.png</href>
      </Icon>
    </IconStyle>
  </Style>

EOF

	rowIndex=1
	last_line=$(wc -l < $directory/data/decodedOutput/*.csv)
	#echo "Last Line: "$last_line
	while IFS=, read time secMark latitude longitude speed elevation accel_long hex
	do
		#echo $rowIndex
		#echo "Lat: "$latitude
		#echo "Long: "$longitude
		if [[ $latitude == "latitude" ]]; then
			continue
		elif [[ $rowIndex == 1 ]]; then
			currentStyle="start"
			currentName="START"
		elif (( $rowIndex >=  $last_line - 1 )); then
			currentStyle="end"
			currentName="END"
		else
			currentStyle="middle"
			currentName="Point "$rowIndex
		fi
		echo '<Placemark><name>'$currentName'</name><styleUrl>#'$currentStyle'</styleUrl><Point><coordinates>'$longitude,$latitude','$elevation'</coordinates></Point></Placemark>' >> $directory/data/kmlOutput/kmlOutput.kml
	
	((rowIndex++))
	
	done < $directory/data/decodedOutput/*.csv


	echo "</Document>" >> $directory/data/kmlOutput/kmlOutput.kml
	echo "</kml>" >> $directory/data/kmlOutput/kmlOutput.kml
}


processing(){
	
	echo
	echo pcap_file_to_read_arg: $pcap_file_to_read_arg
	if [[ $pcap_file_to_read_arg == "" ]]; then
		cd $directory/data
		
		
		echo 
		{ 
			ls *.pcap
		} || { 
			echo
			echo "No pcaps found in data directory" 
			exit 
		}

		while true; do
			echo
			read -rep "Input filename from list: " pcap_file_to_read
			if [ ! -f $pcap_file_to_read ]; then
				echo "    [!!!] File not found!"
			else
				
				pcap_file_to_read=$directory/data/$pcap_file_to_read
				break
			fi
		done

	else

		pcap_file_to_read=$pcap_file_to_read_arg

	fi

	input_pcap_base_name_with_filetype=$(basename $pcap_file_to_read)
	input_pcap_base_name=${input_pcap_base_name_with_filetype%".pcap"}	

	echo input_pcap_base_name: $input_pcap_base_name

	extractPackets
  	getPayload
  	decodePackets
	if [ "$message_type" == "BSM" ]; then
		generateKml
	fi
	echo " "
	echo "----- DECODING COMPLETE -----"
}

processing
