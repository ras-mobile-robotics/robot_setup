# Sourcing
alias sb="source ~/.bashrc"
alias sw="source ~/ros2_ws/install/setup.bash"

# Kill the ROS 2 daemon and restart it
alias rd-restart='ros2 daemon stop; sleep 2; ros2 daemon start'

# Cleans up shared memory segments left behind by Fast DDS
alias fdds-clean='ls /dev/shm/fastrtps* 2>/dev/null | xargs rm -rf && echo "FastDDS Shared Memory Cleaned"'

# Kill Last running job
alias kj="kill -9 %1"

# --- ROS 2 & FZF ---
# Lists
alias frp='ros2 pkg list | fzf'
alias frn='ros2 node list | fzf'
alias frt='ros2 topic list | fzf'
alias frs='ros2 service list | fzf'

# TOPIC UTILITIES
# Fuzzy Echo: Select a topic and start echoing data
fte() {
  local topic=$(ros2 topic list | fzf --header="Select Topic to Echo")
  [ -n "$topic" ] && ros2 topic echo "$topic"
}

# Fuzzy Hz: Check topic frequency with a preview of the message type
fthz() {
  local topic=$(ros2 topic list | fzf --preview "ros2 topic info {}")
  [ -n "$topic" ] && ros2 topic hz "$topic"
}

# NODE & PARAM UTILITIES
# Fuzzy Node Info: Get detailed info on a node
fni() {
  local node=$(ros2 node list | fzf --header="Select Node for Info")
  [ -n "$node" ] && ros2 node info "$node"
}

# Fuzzy Param Get: Two-stage selection (Node -> Parameter)
fpg() {
  local node=$(ros2 node list | fzf --header="Select Node")
  if [ -n "$node" ]; then
    local param=$(ros2 param list "$node" | fzf --header="Select Param from $node")
    [ -n "$param" ] && ros2 param get "$node" "$param"
  fi
}

# BUILD & NAVIGATION (Colcon)
# Fuzzy Colcon Build: Build only the selected package
fcb() {
  local pkg=$(colcon list -n | fzf --header="Package to Build")
  [ -n "$pkg" ] && colcon build --packages-select "$pkg"
}

# Fuzzy ROS CD: Change directory to the source of a package
frcd() {
  local pkg=$(colcon list -n | fzf --header="CD to Package")
  if [ -n "$pkg" ]; then
    local pkg_path=$(colcon list -p --packages-select "$pkg")
    cd "$pkg_path" || echo "Could not find path for $pkg"
  fi
}

# Fuzzy Log Search: Search through the list of log files
frlog() {
  local log_file=$(ls ~/.ros/log | fzf --header="Select Log Folder")
  if [ -n "$log_file" ]; then
     # Attempts to open the latest launch log in that folder
     less +G "~/.ros/log/$log_file/launch.log"
  fi
}

# ROS Debugging
check-ros() {
    echo -e "\e[1;35m================ ROS 2 DEBUG INFO ================\e[0m"
    printf "%-22s \e[1;37m%s\e[0m\n" "ROS Distro:" "$ROS_DISTRO"
    printf "%-22s %s\n" "RMW Implementation:" "${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp (default)}"
    printf "%-22s %s\n" "Domain ID:" "${ROS_DOMAIN_ID:-0}"
    [ -f /home/ubuntu/.turtlebot_id ] && printf "%-22s %02d\n" "Turtlebot ID:" "$(cat /home/ubuntu/.turtlebot_id)"
    
    # Discovery Server Check
    if [ -n "$ROS_DISCOVERY_SERVER" ]; then
        printf "%-22s \e[1;33m%s\e[0m\n" "Discovery Server:" "$ROS_DISCOVERY_SERVER"
    else
        printf "%-22s %s\n" "Discovery Server:" "OFF (Multicast Enabled)"
    fi

    # FastDDS Specifics (Super Client / Local Only)
    if [[ "$FASTRTPS_DEFAULT_PROFILES_FILE" == *"super_client"* ]]; then
        printf "%-22s \e[1;36m%s\e[0m\n" "FastDDS Mode:" "SUPER CLIENT"
    elif [[ "$FASTRTPS_DEFAULT_PROFILES_FILE" == *"local"* ]]; then
        printf "%-22s \e[1;36m%s\e[0m\n" "FastDDS Mode:" "LOCAL ONLY"
    else
        printf "%-22s %s\n" "FastDDS Mode:" "SIMPLE/DEFAULT"
    fi

    echo -e "\n\e[1;34m--- Network Topology ---\e[0m"
    check-net
    echo -e "\e[1;35m==================================================\e[0m"
}

# Network Debugging
check-net() {
    # Column widths
    local w_int=15
    local w_typ=12
    local w_sta=10
    local w_adr=15

    # Print Table Header
    printf "\e[1;34m%-*s %-*s %-*s %-*s\e[0m\n" $w_int "INTERFACE" $w_typ "TYPE" $w_sta "STATE" $w_adr "IP ADDRESS"
    printf "%-*s %-*s %-*s %-*s\n" $w_int "---------" $w_typ "----" $w_sta "-----" $w_adr "----------"

    for dev in /sys/class/net/*; do
        [ -d "$dev" ] || continue
        name=$(basename "$dev")
        [ "$name" == "lo" ] && continue

        # 1. Determine Type
        if [ -d "$dev/wireless" ] || [ -d "/sys/class/net/$name/phy80211" ]; then
            type="Wifi"
        elif [ -d "$dev/bridge" ]; then
            type="Bridge"
        elif [[ "$name" == veth* || "$name" == docker* || "$name" == br-* ]]; then
            type="Virtual"
        else
            type="Ethernet"
        fi

        # 2. Get State & Color
        raw_state=$(cat "$dev/operstate")
        if [ "$raw_state" == "up" ]; then
            state_display="UP"
            color="\e[32m" # Green
        else
            state_display="DOWN"
            color="\e[31m" # Red
        fi

        # 3. Get IP Address
        ip=$(ip -4 addr show "$name" | grep -oP '(?<=inet\s)\d+(\.\d+){3}' | head -n 1)
        [ -z "$ip" ] && ip="---"

        # 4. Print Row (Coloring the state specifically without breaking padding)
        printf "%-*s %-*s ${color}%-*s\e[0m %-*s\n" $w_int "$name" $w_typ "$type" $w_sta "$state_display" $w_adr "$ip"
    done
}

# Ping Discovery Server
check-discovery-server() {
    if [ -z "$ROS_DISCOVERY_SERVER" ]; then
        echo -e "\e[1;31m[!] ROS_DISCOVERY_SERVER is not set.\e[0m"
        echo "Defaulting to localhost for test..."
        local ds_env="127.0.0.1"
    else
        local ds_env="$ROS_DISCOVERY_SERVER"
    fi

    # Extract IP/Hostname for the ping command
    local ds_ip=$(echo "$ds_env" | cut -d':' -f1 | cut -d';' -f1)

    echo -e "\e[1;34m--- ROS 2 Discovery Heartbeat ---\e[0m"
    echo -e "Target Config: \e[1;33m$ds_env\e[0m"
    echo -e "Pinging IP:    \e[1;37m$ds_ip\e[0m"
    echo "---------------------------------"

    # Ping 3 times max with a 1-second timeout per ping
    ping -c 3 -W 1 "$ds_ip"
}


# Sync time
sync-ntp-time() {
    local TICK=$'\u2713'
    local CROSS=$'\u2717'

    echo "Stopping NTP service..."
    sudo timedatectl set-ntp false
    sleep 2

    echo "Starting NTP service and fetching internet time..."
    sudo timedatectl set-ntp true
    
    # "Settle" time: give the background daemon a moment to reach a server
    echo "Settling..."
    sleep 5 

    # Check if successful
    if timedatectl status | grep -q "System clock synchronized: yes"; then
        echo "[$TICK] Time synchronized successfully!"
        date
    else
        echo "[$CROSS] Sync failed. Check internet connection or 'systemd-timesyncd' logs."
    fi
}