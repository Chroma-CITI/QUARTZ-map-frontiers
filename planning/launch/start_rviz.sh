#!/bin/bash

DRONE_NAME=$(rosparam get /drone_name)
NUMBER_OF_DRONES=$(rosparam get /number_of_drones)

PACKAGE_NAME=map_frontiers
SCRIPT_PATH="$(rospack find $PACKAGE_NAME)/scripts/generate_rviz.py"
RVIZ_DIR="$(rospack find $PACKAGE_NAME)/rviz"
OUTPUT_FILE="$RVIZ_DIR/generated.rviz"

# === GENERATE RVIZ CONFIG ===
echo "[*] Generating RViz config for $NUMBER_OF_DRONES robots with base name '$DRONE_NAME'..."

python3 "$SCRIPT_PATH" \
  --count "$NUMBER_OF_DRONES" \
  --namespace "$DRONE_NAME" \
  --base_template "$RVIZ_DIR/base.rviz" \
  --drone_template "$RVIZ_DIR/drone_display_template.rviz" \
  --output "$OUTPUT_FILE"

# Check if generation succeeded
if [ $? -ne 0 ]; then
  echo "[!] Failed to generate RViz config."
  exit 1
fi

# === LAUNCH RVIZ ===
echo "[*] Launching RViz with config: $OUTPUT_FILE"
rosrun rviz rviz -d "$OUTPUT_FILE"

# Wait for all launched processes to complete
wait