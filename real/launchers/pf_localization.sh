#!/bin/bash

source /environment.sh

dt-launchfile-init

VEH="${VEHICLE_NAME:-${ROBOT_NAME:-wolf}}"

dt-exec roslaunch pf_localization_real pf_localization_real.launch veh:="${VEH}"

dt-launchfile-join
