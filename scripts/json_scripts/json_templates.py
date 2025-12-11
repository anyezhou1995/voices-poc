"""
    json_templates.py

    This module provides templates, field maps, and helper functions for creating LandVehicle and 
    TrafficSignalController JSON objects with different coordinate formats.

    Contents:
    - Templates:
        1. base_landVehicle_template: common fields shared by all land vehicle objects.
        2. base_trafficSignalController_template: common fields shared by all traffic signal controller objects.
        3. ltpENU_tspi_template: tspi fields in ltpENU coordinate format.
        4. geocentric_tspi_template: tspi fields in geocentric coordinate format.
        5. geodetic_tspi_template: tspi fields in geodetic coordinate format.
    - Field maps:
        1. ltpENU_field_mappings: maps object data keys to paths in ltpENU_tspi_template.
        2. geocentric_field_mappings: maps object data keys to paths in geocentric_tspi_template.
        3. geodetic_field_mappings: maps object data keys to paths in geodetic_tspi_template
    - Helper functions:
        1. set_nested(): safely sets a value in a nested dictionary.
        2. get_value_at_nested(): retrieves a nested value from a dictionary.
        3. validate_object(): verifies that all required keys are present in the object_data dictionary
        4. remove_null_fields(): removes velocity and acceleration from the object json if they are still at default values
        5. pack_object_data_into_json(): populates templates using a list
            of objects and your specified coordinate format.
        6. get_map_origin_from_scenario(): returns the map origin from the passed in scenario JSON object
"""

import time
import copy
import logging

# ================
# Templates
# ================

# Base LandVehicle Template: contains shared fields except for tspi information
base_landVehicle_template = {
    "attributes": {
        "identifier": "JSON-M-1",
        "designation": "Unset",
        "sourceIdentifier": "Unset",
        "entityID": {
            "attributes": {
                "siteID": 0, 
                "applicationID": 0, 
                "objectID": 0
            }
        },
        "lvcIndicator": "TENA::LVC::LVCindicator_Virtual",
        "entityType": {
            "attributes": {
                "kind": 1, 
                "domain": 1, 
                "country": 225, 
                "category": 81, 
                "subcategory": 10, 
                "specific": 0, 
                "extra": 0
            }
        },
        "affiliation": "TENA::LVC::Affiliation_Friendly",
        "damageState": "TENA::LVC::DamageState_NoDamage",
        "deadReckoningAlgorithm": "TENA::LVC::DeadReckoningAlgorithm_RVW",
        "sensorPedigree": [],
        "trackPedigree": [],
        "landVehicleData" : {
            "attributes" : {
                "exteriorLights" : {
                    "attributes" : {
                        "HeadlightsOn" : False, 
                        "leftTurnSignalOn" : False, 
                        "rightTurnSignalOn" : False, 
                        "hazardSignalOn" : False, 
                        "brakeLightsOn" : False, 
                        "parkingLightsOn" : False, 
                        "specialLightsOn" : False
                    }
                }
            }
        }
    }
}

# Base Traffic Signal Controller Template
base_trafficSignalController_template = {
    "attributes": {
        "identifier": "JSON-TL-1",
        "designation": "Unset",
        "sourceIdentifier": "TL-JSON-1",
        "entityID": {
            "attributes": {
                "siteID": 0, 
                "applicationID": 0, 
                "objectID": 0
            }
        },
        "lvcIndicator": "TENA::LVC::LVCindicator_Virtual",
        "entityType": {
            "attributes": {
                "kind": 5, 
                "domain": 1, 
                "country": 225, 
                "category": 5, 
                "subcategory": 52, 
                "specific": 0, 
                "extra": 0
            }
        },
        "affiliation": "TENA::LVC::Affiliation_Friendly",
        "damageState": "TENA::LVC::DamageState_NoDamage",
        "deadReckoningAlgorithm": "TENA::LVC::DeadReckoningAlgorithm_RVW",
        "sensorPedigree": [],
        "trackPedigree": [],
        "trafficControllerId": "0",
        "trafficSignalPhases": []
    }
}

# ltpENU tspi template: 
ltpENU_tspi_template = {
    "attributes": {
        "time": {
            "attributes": {
               "nanosecondsSince1970": time.time_ns()
            }
        },
        "position": {
            "attributes": {
                "ltpENU_asTransmitted": {
                    "attributes": {
                        "srf": {
                            "attributes": {
                                "rtCode": "TENA::RTCODE_WGS_1984_IDENTITY",
                                "latitudeInDegrees": None, 
                                "longitudeInDegrees": None, 
                                "heightAboveEllipsoidInMeters": None,
                                "azimuthInRadians": 0.0,
                                "xFalseOriginInMeters": 0.0,
                                "yFalseOriginInMeters": 0.0
                            }
                        }, 
                        "xInMeters": None,
                        "yInMeters": None,
                        "zInMeters": None
                    }
                }
            }
        },
        "velocity": {
            "attributes": {
                "ltpENU_asTransmitted": {
                    "attributes": {
                        "srf": {
                            "attributes": {
                                "rtCode": "TENA::RTCODE_WGS_1984_IDENTITY",
                                "latitudeInDegrees": 0.0,
                                "longitudeInDegrees": 0.0,
                                "heightAboveEllipsoidInMeters": 0.0,
                                "azimuthInRadians": 0.0,
                                "xFalseOriginInMeters": 0.0,
                                "yFalseOriginInMeters": 0.0
                            }
                        },
                        "vxInMetersPerSecond": 0.0,
                        "vyInMetersPerSecond": 0.0,
                        "vzInMetersPerSecond": 0.0
                    }
                }
            }
        },
        "acceleration": {
            "attributes": {
                "ltpENU_asTransmitted": {
                    "attributes": {
                        "srf": {
                            "attributes": {
                                "rtCode": "TENA::RTCODE_WGS_1984_IDENTITY",
                                "latitudeInDegrees": 0.0,
                                "longitudeInDegrees": 0.0,
                                "heightAboveEllipsoidInMeters": 0.0,
                                "azimuthInRadians": 0.0,
                                "xFalseOriginInMeters": 0.0,
                                "yFalseOriginInMeters": 0.0
                            }
                        },
                        "axInMetersPerSecondSquared": 0.0,
                        "ayInMetersPerSecondSquared": 0.0,
                        "azInMetersPerSecondSquared": 0.0
                    }
                }
            }
        },
        "orientation": {
            "attributes": {
                "frdWRTltpENUbodyFixedZYX_asTransmitted": {
                    "attributes": {
                        "srf": {
                            "attributes": {
                                "rtCode": "TENA::RTCODE_WGS_1984_IDENTITY", 
                                "latitudeInDegrees": 0.0, 
                                "longitudeInDegrees": 0.0, 
                                "heightAboveEllipsoidInMeters": 0.0, 
                                "azimuthInRadians": 0.0, 
                                "xFalseOriginInMeters": 0.0, 
                                "yFalseOriginInMeters": 0.0
                            }
                        }, 
                        "rotZinRadians": 0.0, 
                        "rotYinRadians": 0.0, 
                        "rotXinRadians": 0.0
                    }
                }
            }
        }
    }
}

# geocentric tspi template
geocentric_tspi_template = {
    "attributes": {
        "time": {
            "attributes": {
                "nanosecondsSince1970": 0
            }
        },
        "position": {
            "attributes": {
                "geocentric_asTransmitted": {
                    "attributes": {
                        "srf": {
                            "attributes": {
                                "rtCode": "TENA::RTCODE_WGS_1984_IDENTITY"
                            }
                        },
                        "xInMeters": 0.0,
                        "yInMeters": 0.0,
                        "zInMeters": 0.0
                    }
                }
            }
        },
        "velocity": {
            "attributes": {
                "geocentric_asTransmitted": {
                    "attributes": {
                        "srf": {
                            "attributes": {
                                "rtCode": "TENA::RTCODE_WGS_1984_IDENTITY"
                            }
                        },
                        "vxInMetersPerSecond": 0.0,
                        "vyInMetersPerSecond": 0.0,
                        "vzInMetersPerSecond": 0.0
                    }
                }
            }
        },
        "acceleration": {
            "attributes": {
                "geocentric_asTransmitted": {
                    "attributes": {
                        "srf": {
                            "attributes": {
                                "rtCode": "TENA::RTCODE_WGS_1984_IDENTITY"
                            }
                        },
                        "axInMetersPerSecondSquared": 0.0,
                        "ayInMetersPerSecondSquared": 0.0,
                        "azInMetersPerSecondSquared": 0.0
                    }
                }
            }
        },
        "orientation": {
            "attributes": {
                "frdWRTgeocentricBodyFixedZYX_asTransmitted": {
                    "attributes": {
                        "srf": {
                            "attributes": {
                                "rtCode": "TENA::RTCODE_WGS_1984_IDENTITY"
                            }
                        },
                        "rotZinRadians": 0.0,
                        "rotYinRadians": 0.0,
                        "rotXinRadians": 0.0
                    }
                }
            }
        }
    }
}

# geodetic tspi template
geodetic_tspi_template = {
    "attributes": {
        "time": {
            "attributes": {
                "nanosecondsSince1970": 0
            }
        },
        "position": {
            "attributes": {
                "geodetic_asTransmitted": {
                    "attributes": {
                        "srf": {
                            "attributes": {
                                "rtCode": "TENA::RTCODE_WGS_1984_IDENTITY"
                            }
                        },
                        "latitudeInDegrees": 0.0,
                        "longitudeInDegrees": 0.0,
                        "heightAboveEllipsoidInMeters": 0.0
                    }
                }
            }
        }
    }
}

# ================
# Field Maps
# ================

# Maps object data keys to paths in the ltpENU tspi template
ltpENU_field_mappings = {
    "x":["attributes", "tspi", "attributes", "position", "attributes", "ltpENU_asTransmitted", "attributes","xInMeters"],
    "y":["attributes", "tspi", "attributes", "position", "attributes", "ltpENU_asTransmitted", "attributes","yInMeters"],
    "z":["attributes", "tspi", "attributes", "position", "attributes", "ltpENU_asTransmitted", "attributes","zInMeters"],
    "vx":["attributes", "tspi", "attributes", "velocity", "attributes", "ltpENU_asTransmitted", "attributes","vxInMetersPerSecond"],
    "vy":["attributes", "tspi", "attributes", "velocity", "attributes", "ltpENU_asTransmitted", "attributes","vyInMetersPerSecond"],
    "vz":["attributes", "tspi", "attributes", "velocity", "attributes", "ltpENU_asTransmitted", "attributes","vzInMetersPerSecond"],
    "ax":["attributes", "tspi", "attributes", "acceleration", "attributes", "ltpENU_asTransmitted", "attributes","axInMetersPerSecondSquared"],
    "ay":["attributes", "tspi", "attributes", "acceleration", "attributes", "ltpENU_asTransmitted", "attributes","ayInMetersPerSecondSquared"],
    "az":["attributes", "tspi", "attributes", "acceleration", "attributes", "ltpENU_asTransmitted", "attributes","azInMetersPerSecondSquared"],
    "yaw":["attributes", "tspi", "attributes", "orientation", "attributes", "frdWRTltpENUbodyFixedZYX_asTransmitted", "attributes","rotZinRadians"],
    "pitch":["attributes", "tspi", "attributes", "orientation", "attributes", "frdWRTltpENUbodyFixedZYX_asTransmitted", "attributes","rotYinRadians"],
    "roll":["attributes", "tspi", "attributes", "orientation", "attributes", "frdWRTltpENUbodyFixedZYX_asTransmitted", "attributes","rotXinRadians"],
}

position_srf_field_mappings = {
    # Map Origin Fields
    "srf_p_latitudeDeg":["attributes", "tspi", "attributes", "position", "attributes", "ltpENU_asTransmitted", "attributes", "srf", "attributes", "latitudeInDegrees"],
    "srf_p_longitudeDeg":["attributes", "tspi", "attributes", "position", "attributes", "ltpENU_asTransmitted", "attributes", "srf", "attributes", "longitudeInDegrees"],
    "srf_p_heightAbvEllip":["attributes", "tspi", "attributes", "position", "attributes", "ltpENU_asTransmitted", "attributes", "srf", "attributes", "heightAboveEllipsoidInMeters"],
    "srf_p_azimuth":["attributes", "tspi", "attributes", "position", "attributes", "ltpENU_asTransmitted", "attributes", "srf", "attributes", "azimuthInRadians"],
    "srf_p_xFalseOrigin":["attributes", "tspi", "attributes", "position", "attributes", "ltpENU_asTransmitted", "attributes", "srf", "attributes", "xFalseOriginInMeters"],
    "srf_p_yFalseOrigin":["attributes", "tspi", "attributes", "position", "attributes", "ltpENU_asTransmitted", "attributes", "srf", "attributes", "yFalseOriginInMeters"] 
}

orientation_srf_field_mappings = {
    # Map Origin Fields
    "srf_o_latitudeDeg":["attributes", "tspi", "attributes", "orientation", "attributes", "frdWRTltpENUbodyFixedZYX_asTransmitted", "attributes", "srf", "attributes", "latitudeInDegrees"],
    "srf_o_longitudeDeg":["attributes", "tspi", "attributes", "orientation", "attributes", "frdWRTltpENUbodyFixedZYX_asTransmitted", "attributes", "srf", "attributes", "longitudeInDegrees"],
    "srf_o_heightAbvEllip":["attributes", "tspi", "attributes", "orientation", "attributes", "frdWRTltpENUbodyFixedZYX_asTransmitted", "attributes", "srf", "attributes", "heightAboveEllipsoidInMeters"],
    "srf_o_azimuth":["attributes", "tspi", "attributes", "orientation", "attributes", "frdWRTltpENUbodyFixedZYX_asTransmitted", "attributes", "srf", "attributes", "azimuthInRadians"],
    "srf_o_xFalseOrigin":["attributes", "tspi", "attributes", "orientation", "attributes", "frdWRTltpENUbodyFixedZYX_asTransmitted", "attributes", "srf", "attributes", "xFalseOriginInMeters"],
    "srf_o_yFalseOrigin":["attributes", "tspi", "attributes", "orientation", "attributes", "frdWRTltpENUbodyFixedZYX_asTransmitted", "attributes", "srf", "attributes", "yFalseOriginInMeters"] 
}

velocity_srf_field_mappings = {
    # Map Origin Fields
    "srf_v_latitudeDeg":["attributes", "tspi", "attributes", "velocity", "attributes", "ltpENU_asTransmitted", "attributes", "srf", "attributes", "latitudeInDegrees"],
    "srf_v_longitudeDeg":["attributes", "tspi", "attributes", "velocity", "attributes", "ltpENU_asTransmitted", "attributes", "srf", "attributes", "longitudeInDegrees"],
    "srf_v_heightAbvEllip":["attributes", "tspi", "attributes", "velocity", "attributes", "ltpENU_asTransmitted", "attributes", "srf", "attributes", "heightAboveEllipsoidInMeters"],
    "srf_v_azimuth":["attributes", "tspi", "attributes", "velocity", "attributes", "ltpENU_asTransmitted", "attributes", "srf", "attributes", "azimuthInRadians"],
    "srf_v_xFalseOrigin":["attributes", "tspi", "attributes", "velocity", "attributes", "ltpENU_asTransmitted", "attributes", "srf", "attributes", "srf_v_xFalseOriginInMeters"],
    "srf_v_yFalseOrigin":["attributes", "tspi", "attributes", "velocity", "attributes", "ltpENU_asTransmitted", "attributes", "srf", "attributes", "yFalseOriginInMeters"] 
}

acceleration_srf_field_mappings = {
    # Map Origin Fields
    "srf_a_latitudeDeg":["attributes", "tspi", "attributes", "acceleration", "attributes", "ltpENU_asTransmitted", "attributes", "srf", "attributes", "latitudeInDegrees"],
    "srf_a_longitudeDeg":["attributes", "tspi", "attributes", "acceleration", "attributes", "ltpENU_asTransmitted", "attributes", "srf", "attributes", "longitudeInDegrees"],
    "srf_a_heightAbvEllip":["attributes", "tspi", "attributes", "acceleration", "attributes", "ltpENU_asTransmitted", "attributes", "srf", "attributes", "heightAboveEllipsoidInMeters"],
    "srf_a_azimuth":["attributes", "tspi", "attributes", "acceleration", "attributes", "ltpENU_asTransmitted", "attributes", "srf", "attributes", "azimuthInRadians"],
    "srf_a_xFalseOrigin":["attributes", "tspi", "attributes", "acceleration", "attributes", "ltpENU_asTransmitted", "attributes", "srf", "attributes", "xFalseOriginInMeters"],
    "srf_a_yFalseOrigin":["attributes", "tspi", "attributes", "acceleration", "attributes", "ltpENU_asTransmitted", "attributes", "srf", "attributes", "yFalseOriginInMeters"] 
}



# Maps object data keys to paths in the geocentric tspi template
geocentric_field_mappings = {
    "x":["attributes", "tspi", "attributes", "position", "attributes", "geocentric_asTransmitted", "attributes","xInMeters"],
    "y":["attributes", "tspi", "attributes", "position", "attributes", "geocentric_asTransmitted", "attributes","yInMeters"],
    "z":["attributes", "tspi", "attributes", "position", "attributes", "geocentric_asTransmitted", "attributes","zInMeters"],
    "vx":["attributes", "tspi", "attributes", "velocity", "attributes", "geocentric_asTransmitted", "attributes","vxInMetersPerSecond"],
    "vy":["attributes", "tspi", "attributes", "velocity", "attributes", "geocentric_asTransmitted", "attributes","vyInMetersPerSecond"],
    "vz":["attributes", "tspi", "attributes", "velocity", "attributes", "geocentric_asTransmitted", "attributes","vzInMetersPerSecond"],
    "ax":["attributes", "tspi", "attributes", "acceleration", "attributes", "geocentric_asTransmitted", "attributes","axInMetersPerSecondSquared"],
    "ay":["attributes", "tspi", "attributes", "acceleration", "attributes", "geocentric_asTransmitted", "attributes","ayInMetersPerSecondSquared"],
    "az":["attributes", "tspi", "attributes", "acceleration", "attributes", "geocentric_asTransmitted", "attributes","azInMetersPerSecondSquared"],
    "yaw":["attributes", "tspi", "attributes", "orientation", "attributes", "frdWRTgeocentricBodyFixedZYX_asTransmitted", "attributes","rotZinRadians"],
    "pitch":["attributes", "tspi", "attributes", "orientation", "attributes", "frdWRTgeocentricBodyFixedZYX_asTransmitted", "attributes","rotYinRadians"],
    "roll":["attributes", "tspi", "attributes", "orientation", "attributes", "frdWRTgeocentricBodyFixedZYX_asTransmitted", "attributes","rotXinRadians"]
}

# Maps object data keys to paths in the geodetic tspi template
geodetic_field_mappings = {
    "latitude":["attributes", "tspi", "attributes", "position", "attributes", "geodetic_asTransmitted", "attributes", "latitudeInDegrees"],
    "longitude":["attributes", "tspi", "attributes", "position", "attributes", "geodetic_asTransmitted", "attributes", "longitudeInDegrees"],
    "height":["attributes", "tspi", "attributes", "position", "attributes", "geodetic_asTransmitted", "attributes", "heightAboveEllipsoidInMeters"]
}

# ================
# Helper Functions
# ================

def set_nested(d, path, value):
    """
        Set a value in a nested dictionary at a location specified by a list of keys.

        Walks through the nested dictionary 'd' following the sequence of keys in 'path'.
        If intermediate dictionaries do not exist, they are created.
        The final key in 'path' is assigned the given 'value'

        Parameters
        ----------
        d : dict
            The dictionary to modify.
        path : list of str
            A list of keys specifying the path where the value should be set.
            The last key in the list is the target key for 'value'.
        value : any
            The value to set at the specified location.

        Returns
        -------
        None
            The dictionary 'd' is modified in place.
    """
    # Traverse the path, creating intermediate dictionaries if needed
    for key in path[:-1]:
        #If the key doesn't exist, set it to an empty dict; then move into it
        d = d.setdefault(key,{})
    # Set the final key to the given value
    d[path[-1]] = value

def get_value_at_nested(data, path):
    """
        Retrieves a nested value from a JSON using a list of keys.

        Traverses the passed in object JSON "data" using "path" to retrieve a value.

        Parameters
        ----------
        data : JSON (dictionary)
            object data
        path : list[string]
            the path to the desired value

        Returns
        -------
        data
            value at the specified path
    """
    for key in path:
        if isinstance(data, dict) and key in data:
            data = data[key]
        else:
            return None
    return data

def validate_object(object_data):
    """
        Validates that each object passed in for JSON creation contains the minimum amount of information needed to create the object JSON.

        Verifies that every object the user wants to create an object JSON for contains the minimum required fields of:
            - "role_name" : the name of the object
            - "x" : the X value of the object's position
            - "y" : the Y value of the object's position
            - "z" : the Z value of the object's position
            - "yaw" : the yaw of the object's orientation
            - "pitch" : the pitch of the object's orientation
            - "roll" : the roll of the object's orientation

        Further improvements can be made to this function to verify fields based on coordinate_format or object type, as well as value and type validations.

        Parameters
        ----------
        object_data : dict
            The object being validated

        Returns
        -------
        Boolean
            True if the object contains all required fields

    """
    required_fields = ["role_name", "x", "y", "z", "yaw", "pitch", "roll"]

    object_keys = list(object_data.keys())

    for field in required_fields:
        if field not in object_keys:
            logging.error(f"Validation failed. '{field}' is missing from object: {object_data}")
            return False

    return True

def remove_null_fields(object_json, field_map):
    """
        Removes velocity and acceleration from the object JSON when values are default.

        Checks the x,y,z velocity and acceleration values in the object_json. If all of these values are still at their default value of 0,
        this function removes those sections from the JSON

        Paramters
        ---------
        object_json : JSON (dictionary)
            JSON of the object being evaluated
        field_map : dict
            key mappings used to get the path to a desired field of the JSON object

    """

    velocities = (get_value_at_nested(object_json, field_map["vx"]), get_value_at_nested(object_json, field_map["vy"]), get_value_at_nested(object_json, field_map["vz"]))
    accelerations = (get_value_at_nested(object_json, field_map["ax"]), get_value_at_nested(object_json, field_map["ay"]), get_value_at_nested(object_json, field_map["az"]))
    
    if velocities == (0.0, 0.0, 0.0):
        object_json["attributes"]["tspi"]["attributes"].pop("velocity", None)
    if accelerations == (0.0, 0.0, 0.0):
        object_json["attributes"]["tspi"]["attributes"].pop("acceleration", None)

    return

def pack_object_data_into_json(object_data_list, entityMap, map_origin, coordinate_format, entity_id):
    """
        Create JSON messages for a list of objects using the templates defined above

        For each object in 'object_data_list' this function:
        - Determines the type of object: (LandVehicle, or TrafficSignalController)
        - Selects the appropriate templates based on 'coordinate_format'
        - Populates the template fields with data from the object and the provided entity_id
        - Returns a list of fully populated JSON dictionaries

        Parameters
        ----------
        object_data_list : list of dict
            List of objects to create JSON messages for. Each object is expected to
            contain fields corresponding to the chosen coordinate format.
        entityMap : dict
            Dictionary containing each object the user is packaging data for. Stores the objectID used in JSON creation
        map_oigin : dict
            Dictionary containing map origin data (latitude, longitude, heightAboveEllipsoid, ...)
        coordinate_format : str
            The coordinate format to use. Must be either "ltpENU" or "geocentric"
        entity_id : str
            The identifier of the source sending the JSON messages

        Returns
        -------
        object_data_json_list : list of dict
            A list of populated JSON dictionaries, one per object in 'object_data_list'
    """
    # Create the list of JSON objects to be returned
    object_data_json_list = []

    # Determines which tspi template to use based on 'coordinate_format'
    logging.debug(f"Using {coordinate_format} tspi template and field mappings for all objects")
    if coordinate_format == "ltpENU":
        tspi_template = ltpENU_tspi_template
        # check if vx, vy, and vz exist in any of the data objects
        tspi_field_map = {**ltpENU_field_mappings,**position_srf_field_mappings, **orientation_srf_field_mappings}
        has_vel = all(all(k in obj for k in ("vx", "vy", "vz")) for obj in object_data_list)
        has_accel = all(all(k in obj for k in ("ax", "ay", "az")) for obj in object_data_list)
        if has_vel:
            tspi_field_map |= velocity_srf_field_mappings
        if has_accel:
            tspi_field_map |= acceleration_srf_field_mappings


    elif coordinate_format == "geocentric":
        tspi_template = geocentric_tspi_template
        tspi_field_map = geocentric_field_mappings
    elif coordinate_format == "geodetic":
        tspi_template = geodetic_tspi_template
        tspi_field_map = geodetic_field_mappings
    else:
        raise ValueError(f"Unknown coordinate format: {coordinate_format}")

    # Create a JSON for each object in 'object_data_list', then appends it to our list
    for object_data in object_data_list:
        # Validate object has all required fields - skip if not valid
        if not validate_object(object_data):
            continue
        # Add map origin to the object_data, and determine the type of object
        # object_data = {**object_data, **map_origin}

        srf_key_map = {
            "latitudeDeg": ("srf_p_latitudeDeg","srf_o_latitudeDeg","srf_v_latitudeDeg","srf_a_latitudeDeg"),
            "longitudeDeg": ("srf_p_longitudeDeg","srf_o_longitudeDeg","srf_v_longitudeDeg","srf_a_longitudeDeg"),
            "heightAbvEllip": ("srf_p_heightAbvEllip","srf_o_heightAbvEllip","srf_v_heightAbvEllip","srf_a_heightAbvEllip"),
            "azimuth": ("srf_p_azimuth","srf_o_azimuth","srf_v_azimuth","srf_a_azimuth"),
            "xFalseOrigin": ("srf_p_xFalseOrigin","srf_o_xFalseOrigin","srf_v_xFalseOrigin","srf_a_xFalseOrigin"),
            "yFalseOrigin": ("srf_p_yFalseOrigin","srf_o_yFalseOrigin","srf_v_yFalseOrigin","srf_a_yFalseOrigin"),
        }
        for origin_key, srf_keys in srf_key_map.items():
            if origin_key not in map_origin:
                logging.warning(f"Map origin missing: {origin_key}")
                continue
            for srf_key in srf_keys:
                if srf_key in tspi_field_map:  # only if that mapping is active
                    object_data[srf_key] = map_origin[origin_key]

        logging.debug(f'object_data: {object_data}')


        object_type = object_data["object_type"]
        
        # Determines which object template to use for the current object
        logging.debug(f"Using {object_type} base template")
        if object_type == "LandVehicle":
            object_data_json = copy.deepcopy(base_landVehicle_template)
        elif object_type == "TrafficSignalController":
            object_data_json = copy.deepcopy(base_trafficSignalController_template)

        else:
            raise ValueError(f"Unknown object type: {object_type}")

        # Populate common fields of the object templates
        object_data_json["attributes"]["identifier"] = object_data["role_name"] #This will need to be in object data
        object_data_json["attributes"]["entityID"]["attributes"]["siteID"] = int(entity_id.split(':')[0])
        object_data_json["attributes"]["entityID"]["attributes"]["applicationID"] = int(entity_id.split(':')[1])
        object_data_json["attributes"]["entityID"]["attributes"]["objectID"] = entityMap[object_data["role_name"]]["objectID"]
        # Insert the tspi template to our base template and begin populating the values
        object_data_json["attributes"]["tspi"] = copy.deepcopy(tspi_template)
        object_data_json["attributes"]["tspi"]["attributes"]["time"]["attributes"]["nanosecondsSince1970"] = time.time_ns()

        # Populate existing tspi fields using the current object data
        for key, path in tspi_field_map.items():
            if key in object_data:
                set_nested(object_data_json, path, object_data[key])

        # Removes velocity and acceleration fields from tspi if they are the default values of 0
        remove_null_fields(object_data_json, tspi_field_map)

        # Add the current object JSON to the list
        object_data_json_list.append(object_data_json)
        logging.debug(f"Added JSON to list: {object_data_json}")

    return object_data_json_list

def get_map_origin_from_scenario(scenario_json):
    """
        Return a dictionary containing the map origin.

        Accesses the 'scenario_json' dictionary object to pull out the map origin latitude, 
        longitude, and height above ellipsoid in meters.

        Parameters
        ----------
        scenario_json : dict
            JSON object of a scenario, containing map origin data

        Returns
        -------
        map_origin
            A dictionary containing the map origin.
    """
    map_origin = {}

    # Store the latitude, longitude, and heightAboveEllipsoid in a dictionary to be returned
    map_origin["latitudeDeg"] = scenario_json["attributes"]["mapGeoReferencePosition"]["attributes"]["geodetic_asTransmitted"]["attributes"]["latitudeInDegrees"]
    map_origin["longitudeDeg"] = scenario_json["attributes"]["mapGeoReferencePosition"]["attributes"]["geodetic_asTransmitted"]["attributes"]["longitudeInDegrees"]
    map_origin["heightAbvEllip"] = scenario_json["attributes"]["mapGeoReferencePosition"]["attributes"]["geodetic_asTransmitted"]["attributes"]["heightAboveEllipsoidInMeters"]
    # These values are currently not populated in the scenario file, setting to default 0 with potential for future support
    map_origin["azimuth"] = 0.0
    map_origin["xFalseOrigin"] = 0.0
    map_origin["yFalseOrigin"] = 0.0

    return map_origin