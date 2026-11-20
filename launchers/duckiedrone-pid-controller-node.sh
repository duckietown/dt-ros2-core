#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# launching app
# background jobs ignore SIGINT; restore it so a stop reaches the node, which lands first
dt-exec env --default-signal=INT ros2 launch --noninteractive pid_controller pid_controller_node.launch.py \
    robot_name:="${VEHICLE_NAME}"

# keep waiting until the controller has landed and exited
while [ -n "$(jobs -pr)" ]; do wait; done


# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
