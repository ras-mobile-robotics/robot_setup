#!/bin/bash
# Run this on your master Robot before copying over to other robots

# 1. Clean cloud-init
sudo cloud-init clean --logs

# 2. Clear netplan leases
echo "--> Clearing netplan leases..."
sudo rm /var/lib/NetworkManager/internal*.lease

# 3. Wipe machine-id so it regenerates on student boot
# Using 'uninitialized' allows the system to generate a truly fresh ID on boot
echo "--> Wiping unique Machine-IDs..."
sudo truncate -s 0 /etc/machine-id
sudo rm -f /var/lib/dbus/machine-id
sudo ln -sf /etc/machine-id /var/lib/dbus/machine-id

# 4. Remove SSH Host keys
# This ensures each robot generates its own unique SSH identity.
echo "--> Removing SSH host keys..."
sudo rm /etc/ssh/ssh_host_*

# 5. Copies a sample wifi_config which has the AP mode config as well
# NOTE: the AP does not have a bot number. That would be generated using the "generate_configs.py"
# THis is required for changing to AP mode or changing Wifi using change_wifi.sh
cp wifi_configs_sample.json /home/ubuntu/wifi_configs.json

# 6. Clear Shell History, logs and APT cache to save space
echo "--> Shrinking image size (logs and cache)..."
history -c && history -w && cat /dev/null > ~/.bash_history
sudo apt-get clean
sudo find /var/log -type f -exec truncate -s 0 {} \;

echo "====================================================="
echo "DONE: Robot SD Prepared."
echo "ACTION: Power off NOW."
echo "====================================================="
