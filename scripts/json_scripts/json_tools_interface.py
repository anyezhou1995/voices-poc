"""
    json_transmitter.py

    Template script demonstrating how to send and receive object SDOs in JSON format using threading for concurrent operations.
    This script is intended to serve as a starting point for distributed-testing event participants utilizing the JSON Tools applications
    to send and receive object SDOs.

    Configuration:
    - The script includes several global variables that can be customized to suit the user's environment and needs.
    
    Helper functions:
    - waypoint(): Reads a CSV file of waypoints and creates a list of dictionary objects (waypoints) for determining location.
    - get_object_data_from_waypoints(): Utilizes the distance traveled retrieved from a custom vehicle algorithm, and the waypoints
        from the CSV file to determine the location of your vehicle. Custom logic is needed for vehicle algorithm, determining location using waypoints,
        and packing object data.
    - get_object_data_from_api(): Returns a dictionary containing object data that has been retrieved via API and transformed to match the necessary structure.
    - wait_for_port(): Wait for a TCP port on a host to become available.
    - transmit_object_json(): Transmit object SDOs to the Publisher REST API.

    Example functions:
    - get_count_of_objects(): Returns a running count of the number of JSONs received for each entity type

    Main threaded functions:
    - transmit_main(): Runs in a thread to continuously:
        1. Retrieve object data (via waypoints or API, depending on configuration)
        2. Package the data into SDOs in JSON format using json_templates
        3. Transmit the SDOs via REST API to the JSON Publisher
    - receive_main(): Runs in a thread to:
        1. Open a socket and bind to a specified host and port
        2. List for incoming SDOs from the JSON Streamer
        3. Print incoming SDOs to the console (From this point the user would modify this section to get the data into their application)

    Main features:
    - Supports multiple coordinate formats (e.g., geocentric, ltpENU)
    - Supports multiploe object types (e.g., LandVehicle, TrafficSignalController)
    - Concurrent transmission and reception using threading.
    - Configurable data source (Waypoints or API)

    Usage:
    - Configure global variables at the start of the script to match your enviornment
    - Customize or extend the helper functions to get object data to/from your application in the format expected by the script and templates.
    - Run the script using "python3 ~/distributed-testing/scripts/json_scripts/json_transmitter.py". Two threads will start:
        1. One for transmittion object SDOs to the JSON Publisher
        2. One for receiving object SDOs from the JSON Streamer
"""

import csv
import time
import math
import requests
import socket
import threading
import logging
import json
import sys
import queue
import os

import fake_api
import json_templates

# =================================
# Global Configurations
# =================================
LOCAL_ADDRESS = os.environ['VUG_LOCAL_ADDRESS']
SELECTED_WAYPOINT_CSV = "delave_waypoints.csv" # CSV file containing waypoint data
GET_OBJECT_TYPE = "waypoints"  # options are "waypoints" or "api"
COORDINATE_FORMAT = "ltpENU" # options are "geocentric" or "ltpENU"
PUBLISHER_IP = LOCAL_ADDRESS
PUBLISHER_ENDPOINT = LOCAL_ADDRESS + ":8004" #IP for TENA Publisher connection
STREAMER_IP = LOCAL_ADDRESS
STREAMER_DATA_ENDPOINT = LOCAL_ADDRESS + ":8005"
DATA_UPDATE_PERIOD = 0.1 # Period for retrieval of data (in seconds)
INDIVIDUAL_TRANSMIT_DELAY = 0.002 # Period between transmission of individual entities to the REST API
LOGGING_LEVEL = logging.DEBUG # sets the minimum level to log [DEBUG, INFO, WARNING, ERROR, CRITICAL]

ENTITY_ID = "03:05" # Unique identifier used to determine entityID of the object JSON
VEHICLE_LIST = {    # List of vehicles you will be providing updates for
    "JSON-M-1" : {
        "objectID" : 1 
    },
    "JSON-M-2" : {
        "objectID" : 2
    },
    "JSON-M-3" : {
        "objectID" : 3
    },
    "JSON-M-4" : {
        "objectID" : 4
    },
    "JSON-M-5" : {
        "objectID" : 5
    }
} 
TRAFFICSIGNALCONTROLLER_LIST = {} # List of traffic signal controllers you will be providing updates for from the API

# =================================
# Logging Setup
# =================================
def setup_logging(level=logging.INFO):
    """
        Creates a new file and sets the execution to log to this file

        This function creates the directory for logging from this script if it doesn't already exist, creates a new
        log file for the current execution of the script, modifies permissions of the directory, and sets the level of
        logging for the current execution.

        Parameters
        ----------
        level : logging
            Minimum level of logging that will be captured in the log file. Default is INFO
    """

    # Gets the path to save your logs to, creates it if it doesn't already exist. ".../distributed-testing/logs/json_script_logs/"
    dt_path = os.getenv("VUG_LOCAL_DT_PATH")
    if not dt_path:
        raise ValueError("Environment variable VUG_LOCAL_DT_PATH not set")
    log_folder = os.path.join(dt_path, "logs/json_script_logs/")
    os.makedirs(log_folder, exist_ok=True)
    os.chmod(log_folder, 0o777) # Modifies permission on the ../distributed-testing/logs/json_script_logs folder to rwxrwxrwx

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    logging.basicConfig(
        filename = (log_folder + f"jsonScriptSend_{timestamp}.log"),
        level=level, # minimum level to log
        format="%(asctime)s [%(levelname)s] [%(threadName)s] %(message)s"
    )
    logging.info("Logging Established")

# =================================
# Helper Functions
# =================================

def waypoint(waypoint_list):
    """
        Parses a CSV file containing waypoint data and returns a list of waypoints.

        Parameters
        ----------
        waypoint_list : str
            Path to the CSV file containing waypoint data.

        Returns
        -------
        list of dict
            List of waypoints, each represented as a dictionary

        Notes
        -----
        Dictionary values should be modified to match the headers in your CSV file
    """
    # Opens the passed in 'waypoint_list' CSV file
    with open(waypoint_list, mode='r', encoding='utf-8') as file:
        csv_reader = csv.DictReader(file)
        # Returns a list of dictionaries, where each row in 'waypoint_list' is an entry in the returned list
        return [
            {
                # The left side are the keys in the resulting dictionary
                # These should not change as they match the expected keys for the json_templates
                # The right side is the corresponding column name in your specified waypoint CSV file
                # These fields will need to be updated based on your waypoint file
                'total_distance_traveled': float(row['distance_traveled_m']),
                'x': float(row['x']),
                'y': float(row['y']),
                'z': float(row['z']),
                'yaw': math.radians(float(row['yaw'])) % (2 * math.pi),
                'pitch': math.radians(float(row['pitch'])) % (2 * math.pi),
                'roll': math.radians(float(row['roll'])) % (2 * math.pi),
            }
            for row in csv_reader
        ]

def get_object_data_from_waypoints(waypoint_data):
    """
        Generate object data based on waypoints and a system control algorithm

        This function takes a list of waypoint dictionaries (from waypoint()) and utilizes some custom progression algorithm
        to determine the object's current state. The algorithm should compute the total distance traveled by the object and then
        the function uses that to determine the object's location with respect to the provided waypoints.

        Users MUST modify this function to fit their simulation or real-world system.
        For example:
        - Use the nearest waypoint already passed (default example implementation)
        - Interpolate between waypoints for smoother motion
        - Apply a custom motion model for velocity/acceleration updates

        Parameters
        ----------
        waypoint_data : list of dict
            Waypoint data, typically loaded from a CSV file

        Returns
        -------
        list of dict
            A list with an object dictionary representing the current state of that object
            The dictionary keys should be those expected by the field mappings in json_templates.py

        Notes
        -----
        - The default implementation uses a simple distance accumulator to determine the total distance traveled
        - The default implementation rounds down to the nearest waypoint and sends that as the object's current location
    """

    # Determine current object state using the user's custom vehicle algorithm - this can include current location, speed, acceleration, update period (etc.).
    # User must determine and retrieve the total distance traveled by the object to determine where along the waypoints their object currently resides

    #----- TODO: YOUR VEHICLE ALGORITHM HERE -----

    #----- Our Example Implementation -----
    total_distance = fake_api.vehicle_algorithm()
    logging.debug(f"Total Distance Travelled: {total_distance}")
    

    # Once you find the distance traveled, you need to move that distance along the path created by the waypoints.
    # This will likely land you inbetween two waypoints.
    # Use this new_linear_distance to figure out which waypoints you fall between
    # Use the logic you decided on in the previous step to select/generate waypoint data for your new location

    #----- TODO: YOUR SELECT/GENERATION WAYPOINT DATA LOGIC HERE -----

    #----- Our Example Implementation -----
    last_waypoint = waypoint_data[0]
    # to keep our place on the waypoints, we want to remove all waypoints before and including the selected waypoint
    while waypoint_data and total_distance > waypoint_data[0]['total_distance_traveled']:
        last_waypoint = waypoint_data.pop(0)
    # Because we remove the last waypoint before our current location, make sure to re-add it.
    #   otherwise element [0] will always be the next waypoint
    waypoint_data.insert(0, last_waypoint)
    logging.debug(f"Last waypoint passed: {last_waypoint}")


    # Pack object data into a dictionary, which will eventually be packed into a JSON
    # Dictionary keys should match those for the appropriate field_mapping object in json_templates.py
    # Not all fields are required, the json_templates will use default values for missing fields.
    # Role_name, (x,y,z) location, and (yaw,pitch,roll) orientation are minimum

    #----- TODO: YOUR OBJECT DATA TRANSFORMATION AND PACKAGING LOGIC HERE -----

    #----- Our Example Implementation -----
    object_data = last_waypoint # Sets object data as the last_waypoint passed
    
    
    object_data["role_name"] = next(iter(VEHICLE_LIST)) # Assigns the name of your vehicle to the first vehicle in VEHICLE_LIST    
    object_data["object_type"] = "LandVehicle" # Sets the object_type as LandVehicle, used by the JSON packaging function to determine which JSON template to use
    logging.debug(f"Waypoint following vehicle update: {object_data}")
    return [object_data] # Return object_data in a list

def get_object_data_from_api(vehicle_list, trafficSignalController_list):
    """
        Generate object data using information pulled from your application's API

        This function will retrieve object data from your specific API. 
        This must include the object's role_name, (x,y,z) location, and (yaw,pitch,roll) orientation at a minimum.

        Users MUST modify this function to fit their API.

        Parameters
        ----------
        vehicle_list : list of str
            List of vehicle role_names the user is getting object data for
        trafficSignalController_list : list of str
            List of trafficSignalController role_names the user is getting object data for

        Returns
        -------
        object_data_list : list of dict
            A list with object dictionaries representing the current state of each object
            The dictionary keys should be those expected by the field mappings in json_templates.py

        Notes
        -----
        - The default implementation uses 'fake_api.py' to get a list of dictionaries containing vehicle information.
        - The default implementation performs some modification to ensure object data matches what is expected by the 
            field mappings in json_templates.py (e.g., re-naming dictionary keys with <object>.pop("<old_keyName>"), converting degrees to radians, etc.)
    """

    # Final list of objects to be returned
    object_data_list = []

    # Retrieve vehicle object data via API - ensure to get all required fields
    #----- TODO: RETRIEVE VEHICLE DATA FROM API LOGIC HERE -----

    #----- Our Example Implementation -----
    api_vehicle_data = fake_api.get_vehicles() #Returns a list of dictionaries containing vehicle data
    
    logging.debug("Vehicle data retrieved from API")
    # Modify vehicle object data to match the format expected by json_templates
    # Additionally, modify values based on alternative coordinate systems (i.e. converting into EastNorthUp)
    #----- TODO: TRANSFORM DATA RECEIVED BY API LOGIC HERE ------

    #----- Our Example Implementation -----
    for vehicle in api_vehicle_data:
        if vehicle["vehicle_name"] in vehicle_list:
            vehicle["role_name"] = vehicle.pop("vehicle_name") # Renames the keys of the vehicle dictionary in place to what is expected by the JSON packaging function
            vehicle["x"] = vehicle.pop("xPosition")
            vehicle["y"] = vehicle.pop("yPosition")
            vehicle["z"] = vehicle.pop("zPosition")
            vehicle["yaw"] = math.radians(vehicle.pop("yawDeg")) # Renames the keys in place and converts degrees to radians
            vehicle["pitch"] = math.radians(vehicle.pop("pitchDeg"))
            vehicle["roll"] = math.radians(vehicle.pop("rollDeg"))
            # The below line is an example of converting data from a left hand 3d coordinate space into a right hand 3d coordinate space
            # vehicle["roll"] = math.radians(vehicle.pop("rollDeg")) + math.pi 
            
            # The below line is needed to ensure that the correct template is used during JSON creation
            vehicle["object_type"] = "LandVehicle"
            object_data_list.append(vehicle)

    logging.debug("Vehicle data transformed")
    # Retrieve traffic signal controller object data via API - ensure to get all required fields
    #----- TODO: RETRIEVE TRAFFIC SIGNAL CONTROLLER DATA FROM API LOGIC HERE -----

    #----- Our Example Implementation -----
    api_tsc_data = fake_api.get_trafficSignalControllers()

    logging.debug("Traffic Signal Controller data retrieved from API")
    # Modify traffic signal controller data to match the format expected by json_templates
    #----- TODO: TRANSFORM DATA RECEIVED BY API LOGIC HERE -----

    #----- Our Example Implementation
    for tsc in api_tsc_data: 
        if tsc["id"] in trafficSignalController_list:
            tsc["object_type"] = "TrafficSignalController"
            object_data_list.append(tsc)

    logging.debug("Traffic Signal Controller data modified")

    logging.debug(f"Object updates from API: {object_data_list}")

    return object_data_list

def print_objects_to_console(object_data_list, stdout_lock):
    """
        Prints an update to the console for each object being sent to the JSON Publisher

        This function will print continuously updating lines to the bottom of the console for updates to each
        object being sent to the JSON Publisher. It uses a lock to ensure it is not writing to the console at the same
        time as the receive thread

        Parameters
        ----------
        - object_data_list : list
            objects being sent to the JSON Publisher
        - stdout_lock : lock
            threading.lock to prevent multiple threads accessing the console at the same time
    """
    with stdout_lock:
        sys.stdout.write(f"\033[s") # save cursor location
        sys.stdout.write(f"\033[999B") # more cursor to the bottom of the console
        sys.stdout.write(f"\033[{len(object_data_list)+2}F") # move up number of objects + header + spacer lines
        sys.stdout.write(f"\033[J") # clear dashboard area

        print("Object Updates To Send")
        for object in object_data_list: # print each object
            print(f"Object Name: {object['role_name']} | Object Type: {object['object_type']} | XPosition: {object['x']} | YPosition: {object['y']} | ZPosition: {object['z']}")
        print()

        sys.stdout.write(f"\033[u") # restore cursor location
        sys.stdout.flush()

def wait_for_port(host, port, timeout=30):
    """
        Wait for a TCP port on a host to become available.

        This function repeatedly attempts to create a socket connection to the specified 
            host and port until it succeeds or the timeout is reached.
        It is used to wait for the REST API to come online before proceeding

        Parameters
        ----------
        host : str
            Hostname or IP address of the REST API
        port : int
            TCP port number to check
        timeout : int, optional
            Maximum time in seconds to wait before giving up (default is 30).

        Returns
        -------
        bool
            True if the connection was successful within the timeout period
            False if the timeout expired without success

        Notes
        -----
        - Uses a 2-second socket timeout for each connection attempt
        - Retries once per second until the total timeout is reached
    """
    # Gets the time that connection attempts begin
    start = time.time()
    # Determines if the timeout has been reached
    while time.time() - start < timeout:
        # Attempts to create a connection at the specified host:port
        try:
            with socket.create_connection((host, port), timeout=2):
                return True
        except OSError:
            time.sleep(1)
    return False

def transmit_object_json(object_json_list, EntityMap, session: requests.Session):
    """
        Transmit object SDOs to the Publisher REST API.

        For each object SDO in the list:
        - If the object has not been seen before (not in 'EntityMap'), a POST request
            is made to create it on the publisher. The returned 'sdo_index' is used to 
            construct the unique object URL, which is stored in 'EntityMap'
        - If the object already exists (in 'EntityMap'), a PUT request is made to update
            it at its previously assigned URL

        A connection check ('wait_for_port') ensures the publisher service is available
            before any requests are sent

        Parameters
        ----------
        object_json_list : list of dict
            A list of JSON-compatible dictionaries representing object SDOs to transmit
        EntityMap : dict
            A dictionary mapping entity names to their publisher URLs
            Updated in-place as new entities are created

        Returns
        -------
        None

    """
    

    for object_json in object_json_list:
        # Pulls the EntityName from the object
        EntityName = object_json["attributes"]["identifier"]

        # Set the object and entity type for the publisher url
        if object_json.get("attributes",{}).get("landVehicleData") is not None:
            object_type = 'DOT_OSTR-LandVehicle-v1.1.2'
            entity_type = 'DOT_OSTR::Entities::LandVehicle' 
        elif object_json.get("attributes",{}).get("trafficSignalPhases") is not None:
            object_type = 'DOT_OSTR-TrafficSignalController-v1.3.4'
            entity_type = 'DOT_OSTR::Entities::TrafficSignalController'
        
        tena_publisher_url = f"http://{PUBLISHER_ENDPOINT}/v1/objects/{object_type}/{entity_type}"
        
        logging.info(f"Transmitting {entity_type} {EntityName} to: {tena_publisher_url}")

        # Determines whether a new object (PUSH) request is being sent or update to existing object (PUT)
        if "url" not in EntityMap[EntityName].keys():
            initial_response = session.post(tena_publisher_url, json=object_json)
            # initial_response.raise_for_status()
            sdo_index = initial_response.json().get("sdo_index")
            EntityMap[EntityName]["url"] = f"{tena_publisher_url}/{sdo_index}"
            logging.info(f"Response: {initial_response}")
        else:
            update_response = session.put(EntityMap[EntityName]["url"], json=object_json)
            logging.info(f"Response: {update_response}")
            # update_response.raise_for_status()
        time.sleep(INDIVIDUAL_TRANSMIT_DELAY)

        # this will send to a speficied REST endpoint
    return

def transmit_main(mapOrigin_queue, stdout_lock):
    """
        Main loop for transmitting object SDOs to the JSON Publisher

        This function continuously retrieves object data (either from waypoints or from an API, depending on configuration),
        converts it into JSON messages using the appropriate template, and transmits those SDOs to the Publisher REST API.
        It maintains an internal mapping ('EntityMap') of object IDs to publisher entity URLs so that new objects are 
        created with POST and existing objects are updated with PUT.

        Parameters
        ----------
        mapOrigin_queue : queue
            This queue acts as a shared storage space between the receive thread and the send thread. It is used to
            store map origin data that is received via Scenario JSON and used in object JSON creation.
        stdout_lock : lock
            This lock ensures that the receive and send threads are not accessing the stdout console at the same time
            
        Globals Used
        ------------
        These are set in the "Global configuration" section at the beginning of this script
        GET_OBJECT_TYPE : str
            Source of object data. Must be "waypoints" or "api"
        SELECTED_WAYPOINT_CSV : str
            Path to the CSV file containing waypoint data
        VEHICLE_LIST : list of str
            List of vehicles to pull object data from the API
        TRAFFICSIGNALCONTROLLER_LIST : list of str
            List of traffic signal controllers to pull object data from the API
        COORDINATE_FORMAT : str
            Coordinate system used for packaging object SDOs. Must be "geocentric" or "ltpENU"
        ENTITY_ID : str
            Identifier for the site and application ID sending the object SDOs
        DATA_UPDATE_PERIOD : float
            Period of time in seconds the transmitter will wait between gathering information to create and send object

        Notes
        -----
        - Runs indefinitely until the script is terminated
    """

    # Hashmap of vehicle ID to entity ID and publisher URL
    EntityMap = {**VEHICLE_LIST, **TRAFFICSIGNALCONTROLLER_LIST}

    # Creates the list of waypoints if needed
    if GET_OBJECT_TYPE == "waypoints":
        waypoint_list = waypoint(SELECTED_WAYPOINT_CSV)
        logging.info(f"Waypoints generated from: {SELECTED_WAYPOINT_CSV}")

    # Wait for map origin to be placed in queue from receive thread
    print("Waiting for Map Origin from receive_thread...")
    mapOrigin = mapOrigin_queue.get()
    print(f"Map Origin received from receive_thread: {mapOrigin}")

    # Connection check to ensure REST API is available
    if not wait_for_port(PUBLISHER_IP, 8004):
        logging.error("REST API is not available")
        return
    
    session = requests.Session()
    
    next_t = time.monotonic()
    # Begin loop of retrieving vehicle information
    while True: 

        ##### Step 1: Get object data
        # This can be from either
        #   - an application API or 
        #   - a list of waypoints

        if GET_OBJECT_TYPE == "waypoints":
            object_data_list = get_object_data_from_waypoints(waypoint_list)
        elif GET_OBJECT_TYPE == "api":
            object_data_list = get_object_data_from_api(VEHICLE_LIST, TRAFFICSIGNALCONTROLLER_LIST)
        else:
            logging.error("Invalid Object Type, must be 'waypoints' or 'api'")
            return

        print_objects_to_console(object_data_list, stdout_lock)

        ##### Step 2: pack data into JSON
        object_json_list = json_templates.pack_object_data_into_json(object_data_list, EntityMap, mapOrigin, COORDINATE_FORMAT, ENTITY_ID)

        ##### Step 3: transmit the JSON 
        transmit_object_json(object_json_list, EntityMap, session)

        ##### Step 4: wait for update period and repeat
        next_t += DATA_UPDATE_PERIOD
        time.sleep(max(0, next_t - time.monotonic()))

def get_count_of_objects(type_counts, type_name, stdout_lock):
    """
        Example function for processing received objects.

        This function serves as a simple example of some processing that can be done on received JSON objects.
        This function prints a running count of the number of messages received for each entity type.

        Parameters:
        -----------
        - type_counts: dict
            dictionary containing the number of JSONs received per entity type
        - type_name: string
            entity type to be added to the count
        - stdout_lock: lock
            This lock ensures that the receive and send threads are not accessing the stdout console at the same time
    """
     # Keeps count of each type_name
    if type_name not in type_counts:
        type_counts[type_name] = 0
    type_counts[type_name] += 1
                
    # Build display string
    display_types = []
    for name, count in type_counts.items():
        short_name = name.split("::")[-1]  # Get last part of each type_name after the "::" (ex. DOT_OSTR::Entities::LandVehicle)
        display_types.append(f"{short_name}: {count}")
                
    # Update the same line with current counts
    with stdout_lock:
        sys.stdout.write("\033[s") # save cursor location
        sys.stdout.write("\033[999B") # move cursor to bottom of the console
        sys.stdout.write(f"\r{' | '.join(display_types)}  ") # print count of JSONs received by entity type
        sys.stdout.write("\033[u") # return to saved cursor location
        sys.stdout.flush() # Ensure it prints immediately

def receive_main(mapOrigin_queue, stdout_lock):
    """
        Start a UDP server to receive object SDOs in JSON format

        This function creates a UDP socket, binds it to a configurable host and port, and listens for incoming connections.
        When a client connects, it receives up to 8192 bytes of data, decodes it as UTF-8, and prints it to the console.

        Parameters
        ----------
        mapOrigin_queue : queue
            This queue acts as a shared storage space between the receive thread and the send thread. It is used to
            store map origin data that is received via Scenario JSON and used in object JSON creation.
        stdout_lock : lock
            This lock ensures that the receive and send threads are not accessing the stdout console at the same time
            
        Notes
        -----
        - Runs indefinitely until the script is terminated
        - Prints received data directly to the console
        - Assumes incoming data is UTF-8 encoded text.
        - It is up to the user to implement further processing of the received SDOs
    """

    # Define host and port
    HOST = STREAMER_IP  # Listen on localhost
    PORT = 8005  # Choose an unused port

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    sock.bind((HOST, PORT))
    print(f"Server listening on {HOST}:{PORT} for UDP data...")

    # Initialize counters - dictionary to track all types
    type_counts = {}

    while True:
        data, addr = sock.recvfrom(8192)
        try: 
            json_data = json.loads(data.decode('utf-8'))

            # Pulls the EntityType from the received json
            type_name = json_data["metadata"]["type_name"]

            # Gets the map origin from the scenario json and puts it in a queue for the send thread to use
            if type_name == "DOT_OSTR::Configuration::Scenario":
                map_origin = json_templates.get_map_origin_from_scenario(json_data)
                logging.info(f"Map Origin received from Scenario: {map_origin}")
                mapOrigin_queue.put(map_origin)

            #----- Our Example Implementation -----
            get_count_of_objects(type_counts, type_name, stdout_lock)                            
                         
        except json.JSONDecodeError:
            logging.error(f"Received non-JSON data from {addr}")

def main():
    """
        Start the transmit and receive threads.

        This function creates and starts two threads:
        - One to handle receiving SDOs in JSON format from the JSON Streamer
        - One to handle sending SDOs in JSON format via REST API to the JSON Publisher

        Both threads are started and run concurrently. This function does not return until the script is terminated.
    """
    # Create log file for this execution of the json_tools_interface script
    setup_logging(LOGGING_LEVEL)
    
    # Create a queue for the receive thread to send map origin data from the scenario to the send thread
    mapOrigin_queue = queue.Queue()
    # Create a lock so threads are not accessing stdout at the same time
    stdout_lock = threading.Lock()
    # Link the receive_thread to run the code in the receive_main() function
    receive_thread = threading.Thread(target=receive_main, args=(mapOrigin_queue,stdout_lock), name="ReceiveThread")
    # Link the send_thread to run the code in the transmit_main() function
    send_thread = threading.Thread(target=transmit_main, args=(mapOrigin_queue,stdout_lock), name="SendThread")
    
    # Start both threads
    receive_thread.start()
    send_thread.start()


# Run the main function only if this script is executed directly
if __name__ == "__main__":
    main()
