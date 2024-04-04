#!/bin/bash

carma exec 'source /opt/carma/install_ros2/setup.bash && ros2 service call /guidance/set_guidance_active carma_planning_msgs/srv/SetGuidanceActive "{guidance_active: true}"'