#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# launching app
# dt-exec echo "This is an empty launch script. Update it to launch your application."
# dt-exec rosrun -m "my_package.camera_node"
# dt-exec rosrun -m "my_package.wheel_control_node"
# dt-exec python3 -m "my_package.lane_follower"
dt-exec rosrun Main camera_node.py
dt-exec rosrun Main wheel_control_node.py
dt-exec rosrun Main encoder_node.py

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
