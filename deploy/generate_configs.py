import os
import argparse

def generate_configs():
    parser = argparse.ArgumentParser(description="Generate TurtleBot 4 Raspberry Pi Cloud-Init configs.")
    
    # Required Arguments
    parser.add_argument("--ssid", required=True, help="WIFI SSID")
    parser.add_argument("--password", required=True, help="WIFI Password")
    parser.add_argument("--num", type=int, default=15, help="Number of robots")
    
    # LAN Configuration Arguments
    parser.add_argument("--dhcp-lan", action="store_true", help="Use DHCP for LAN/Ethernet")
    parser.add_argument("--lan-base", default="192.168.50.", help="Base IP for LAN")
    parser.add_argument("--lan-start", type=int, default=201, help="Starting suffix for LAN IP")
    
    # WIFI Configuration Arguments
    parser.add_argument("--wifi-base", default="192.168.50.", help="Base IP for WIFI")
    parser.add_argument("--wifi-start", type=int, default=101, help="Starting suffix for WIFI IP")
    
    # General Network Defaults
    parser.add_argument("--gateway", default="192.168.50.1", help="Gateway IP")

    args = parser.parse_args()

    # Define DNS list: Gateway first, then Google DNS
    dns_list = f"{args.gateway}, 8.8.8.8"

    for i in range(1, args.num + 1):
        bot_id_str = f"{i:02d}"
        bot_id_int = i
        hostname = f"turtlebot{bot_id_str}"
        
        # Calculate Static IPs
        wifi_ip = f"{args.wifi_base}{args.wifi_start + i - 1}"
        lan_ip = f"{args.lan_base}{args.lan_start + i - 1}"
        
        # Determine LAN (eth0) configuration
        if args.dhcp_lan:
            eth0_config = "dhcp4: true"
        else:
            eth0_config = f"""dhcp4: no
    addresses: [{lan_ip}/24]
    gateway4: {args.gateway}
    nameservers:
      addresses: [{dns_list}]"""

        # Folder setup
        folder = f"configs/bot{bot_id_str}"
        os.makedirs(folder, exist_ok=True)

        # 1. Generate user-data
        user_data_content = f"""#cloud-config
hostname: {hostname}
manage_etc_hosts: true
ssh_pwauth: true
chpasswd:
  list: |
    ubuntu:turtlebot
  expire: False

runcmd:
  - echo "{bot_id_str}" >| /home/ubuntu/.turtlebot_id
  - chown ubuntu:ubuntu /home/ubuntu/.turtlebot_id
  - sed -i 's/"ssid": "TurtleBot_AP_"/"ssid": "TurtleBot_AP_{bot_id_int}"/' /home/ubuntu/wifi_configs.json
  - sed -i 's/export ROS_DOMAIN_ID=.*/export ROS_DOMAIN_ID="{bot_id_int}"/' /etc/turtlebot4/setup.bash
  - sed -i 's|export ROBOT_NAMESPACE=.*|export ROBOT_NAMESPACE="/robot_{bot_id_str}"|' /etc/turtlebot4/setup.bash
  - chown ubuntu:ubuntu /home/ubuntu/wifi_configs.json
"""

        # 2. Generate network-config
        network_config_content = f"""version: 2
ethernets:
  eth0:
    {eth0_config}
    dhcp-identifier: mac
wifis:
  wlan0:
    dhcp4: no
    addresses: [{wifi_ip}/24]
    gateway4: {args.gateway}
    nameservers:
      addresses: [{dns_list}]
    access-points:
      "{args.ssid}":
        password: "{args.password}"
"""

        with open(f"{folder}/user-data", "w") as f:
            f.write(user_data_content)
        with open(f"{folder}/network-config", "w") as f:
            f.write(network_config_content)

    print(f"Generated {args.num} configs in /configs/")
    print(f"DNS Servers      : {dns_list}")
    print(f"WIFI Static Range: {args.wifi_base}{args.wifi_start} to {args.wifi_base}{args.wifi_start + args.num - 1}")
    if args.dhcp_lan:
        print("LAN Mode: DHCP")
    else:
        print(f"LAN Static Range : {args.lan_base}{args.lan_start} to {args.lan_base}{args.lan_start + args.num - 1}")

if __name__ == "__main__":
    generate_configs()