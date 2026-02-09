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
        namespace = f"/robot_{bot_id_str}"
        
        # Calculate Static IPs
        wifi_ip = f"{args.wifi_base}{args.wifi_start + i - 1}"
        lan_ip = f"{args.lan_base}{args.lan_start + i - 1}"
        
        # Determine LAN (eth0) configuration
        if args.dhcp_lan:
            eth0_config = "dhcp4: true"
        else:
            eth0_config = f"""dhcp4: no
    addresses: [{lan_ip}/24]
    routes:
      - to: default
        via: {args.gateway}
        metric: 600
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

ssh_genkeytypes: ['rsa', 'ecdsa', 'ed25519']

timezone: America/Phoenix

system_info:
  default_user:
    name: ubuntu

runcmd:
  - test -f /etc/ssh/ssh_host_rsa_key || dpkg-reconfigure openssh-server
  - echo "{bot_id_str}" >| /home/ubuntu/.turtlebot_id
  - chown ubuntu:ubuntu /home/ubuntu/.turtlebot_id
  - sed -i 's/export ROS_DOMAIN_ID=.*/export ROS_DOMAIN_ID="{bot_id_int}"/' /etc/turtlebot4/setup.bash
  - sed -i 's|export ROBOT_NAMESPACE=.*|export ROBOT_NAMESPACE="{namespace}"|' /etc/turtlebot4/setup.bash
  - echo "(1/3) Waiting 120 secs for Create 3 to switch on for the first time..." >| /home/ubuntu/.cloud_init_status
  - sleep 120
  - echo "Verifying firmware version I.0.0.FastDDS is active..."
  - echo "(2/3) Verifying firmware version I.0.0.FastDDS is active..." >| /home/ubuntu/.cloud_init_status
  - bash -c "for i in {{1..20}}; do if ping -c 1 -W 1 192.168.186.2 > /dev/null && curl -sk http://192.168.186.2/api/about | grep -qi 'I.0.0.FastDDS'; then echo 'Create 3 Base I.0.0 found'; break; fi; echo 'Waiting for Base... attempt \$i'; sleep 5; done"
  - echo "Applying ROS 2 Discovery Server Config..."
  - echo "(3/3) Applying ROS 2 Discovery Server Config.." >| /home/ubuntu/.cloud_init_status
  - curl -X POST -d "ros_domain_id={bot_id_int}&ros_namespace={namespace}/_do_not_use&rmw_implementation=rmw_fastrtps_cpp&fast_discovery_server_value=192.168.186.3:11811&fast_discovery_server_enabled=true" http://192.168.186.2/ros-config-save-main
  - sleep 15
  - curl -X POST http://192.168.186.2/api/reboot
  - ros2 daemon stop
  - echo "SETUP COMPLETE!!! WAIT FOR 90 SECS CREATE BASE TO REBOOT!" > /home/ubuntu/.cloud_init_status
"""

        # 2. Generate network-config
        network_config_content = f"""version: 2
renderer: NetworkManager
ethernets:
  eth0:
    {eth0_config}
    dhcp-identifier: mac
    optional: true
  usb0:
    addresses: [192.168.186.3/24]
    dhcp4: no
wifis:
  wlan0:
    dhcp4: no
    addresses: [{wifi_ip}/24]
    routes:
      - to: default
        via: {args.gateway}
        metric: 50
    nameservers:
      addresses: [{dns_list}]
    access-points:
      "{args.ssid}":
        auth:
            key-management: "psk"
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