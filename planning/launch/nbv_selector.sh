#!/bin/bash

# Expecting these two rosparam to be already set
NUM_DRONES=$(rosparam get /number_of_drones)
DRONE_NAME=$(rosparam get /drone_name)

timestamp=$(date +%Y-%m-%d_%H-%M-%S)
bag_name="next_best_viewpoint_data_$timestamp"

echo "Creating $NUM_DRONES nbv selector"

# Loop through the number of drones and launch each
for ((i=0; i<NUM_DRONES; i++)); do
    namespace="/${DRONE_NAME}_${i}"
    roslaunch map_frontiers NBV_selector.launch namespace:=$namespace bag_name:=$bag_name &
done

# Wait for all launched processes to complete
wait