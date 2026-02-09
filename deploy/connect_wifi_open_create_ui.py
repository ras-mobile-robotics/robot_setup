import subprocess
import re
import time
import webbrowser
import argparse
import sys

def find_and_connect():
    # Set up Argument Parsing
    parser = argparse.ArgumentParser(description="Scan and connect to 'Create-.*' networks using wlan1.")
    parser.add_argument('-f', '--force', action='store_true', help="Force connection without asking for confirmation.")
    args = parser.parse_args()

    interface = "wlan1"

    # 1. Scan for available Wi-Fi networks specifically on wlan1
    print(f"Scanning for 'Create-.*' networks on {interface}...")
    try:
        # We specify the interface to ensure wlan0 remains untouched
        scan_output = subprocess.check_output(
            ["nmcli", "-t", "-f", "SSID", "dev", "wifi", "list", "ifname", interface], 
            stderr=subprocess.STDOUT
        ).decode('utf-8')
    except subprocess.CalledProcessError as e:
        print(f"Error: Could not scan on {interface}. Is the adapter plugged in?")
        print(f"Details: {e.output.decode()}")
        return

    # 2. Find all matches and filter out empty strings
    matches = list(set(re.findall(r"(Create-.*)", scan_output)))
    
    if not matches:
        print(f"No network matching 'Create-.*' was found on {interface}.")
        return

    # 3. List matches and select target
    print("\nFound the following matching networks:")
    for i, ssid in enumerate(matches):
        print(f" [{i}] {ssid}")
    
    target_ssid = matches[0]
    print(f"\nTargeting: {target_ssid} via {interface}")

    # 4. Confirmation Logic
    if not args.force:
        confirm = input(f"Proceed with connection to {target_ssid}? (y/n): ").lower()
        if confirm != 'y':
            print("Connection aborted by user.")
            sys.exit()

    # 5. Attempt to connect using the specific interface
    print(f"Connecting {interface} to {target_ssid}...")
    # 'ifname' tells NetworkManager exactly which hardware to use
    connect_cmd = ["nmcli", "device", "wifi", "connect", target_ssid, "ifname", interface]
    
    result = subprocess.run(connect_cmd, capture_output=True, text=True)
    
    if result.returncode == 0:
        print(f"Successfully connected {interface} to {target_ssid}.")
        print("Waiting for IP assignment...")
        time.sleep(4) # Slight increase to ensure wlan1 gets its DHCP lease
        
        url = "http://192.168.10.1"
        print(f"Opening {url}")
        webbrowser.open(url)
    else:
        print(f"Failed to connect: {result.stderr}")

    # 6. Reminders
    print(f"Connection is active on {interface}.")
    print("\n" + "!"*30)
    print("REMINDER:")
    print(f"1. Forget Wifi Connection")
    print("2. Upload firmware (1.0.0 for ROS2 Jazzy FastDDS).")
    print("!"*30)

if __name__ == "__main__":
    find_and_connect()