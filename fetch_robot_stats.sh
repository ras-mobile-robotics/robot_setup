#!/bin/bash

# --- CONFIGURATION ---
TIMEOUT="8s"
STATUS_FILE="/tmp/robot_status"

# !!! IMPORTANT: Match this to your ROS environment !!!
export ROS_DOMAIN_ID=0 
export PYTHONUNBUFFERED=1

# 1. Load ROS 2 Environment
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
fi
source /etc/turtlebot4/setup.bash 2>/dev/null

# 2. Fetch Battery Status
RAW_BATT=$(timeout "$TIMEOUT" ros2 topic echo --once --field percentage /robot_0/battery_state 2>/dev/null)
CLEAN_BATT=$(echo "$RAW_BATT" | tr -d '[:space:]' | sed 's/---//g')
[[ $CLEAN_BATT == .* ]] && CLEAN_BATT="0$CLEAN_BATT"

if [[ $CLEAN_BATT =~ ^[0-9.-]+$ ]]; then
    PERCENT=$(printf "%.0f" "$(echo "$CLEAN_BATT * 100" | bc -l)")
    if [ "$PERCENT" -ge 90 ]; then BATT_ICON="󰁹"; elif [ "$PERCENT" -ge 70 ]; then BATT_ICON="󰂀"; elif [ "$PERCENT" -ge 50 ]; then BATT_ICON="󰁾"; elif [ "$PERCENT" -ge 30 ]; then BATT_ICON="󰁼"; else BATT_ICON="󰂃"; fi
    BATT_STR="$BATT_ICON ${PERCENT}%"
else
    BATT_STR="󰂑 ??"
fi

sleep 1

# 3. Fetch Dock Status
RAW_DOCK=$(timeout "$TIMEOUT" ros2 topic echo --once --field is_docked /robot_0/dock_status 2>/dev/null)
CLEAN_DOCK=$(echo "$RAW_DOCK" | tr -d '[:space:]' | sed 's/---//g' | tr '[:upper:]' '[:lower:]')

if [ "$CLEAN_DOCK" = "true" ]; then
    DOCK_STR="󱐋 Charging"
elif [ "$CLEAN_DOCK" = "false" ]; then
    DOCK_STR="󰑭 Mobile"
else
    DOCK_STR="󰚵 Unknown"
fi

# 4. Write to file
echo "$BATT_STR │ $DOCK_STR" > "$STATUS_FILE"
