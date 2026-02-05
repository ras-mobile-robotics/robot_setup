#!/bin/bash

# TODO: Need to add some other packages
sudo apt update
sudo apt upgrade
sudo apt install bc tmux python3-colcon-clean ntpdate -y

# Copy bashrc, tmux and vim configs
cp ~/.bashrc ~/.bashrc_$(date +%Y%m%d_%H%M%S) 
cp bashrc ~/.bashrc
cp ~/.tmux.conf ~/.tmux_$(date +%Y%m%d_%H%M%S) 
cp tmux.conf ~/.tmux.conf
cp ~/.vimrc ~/.vimrc_$(date +%Y%m%d_%H%M%S) 
cp vimrc ~/.vimrc

# File Permissions
chmod +x ~/robot_setup/fetch_robot_stats.sh

### For Robot Status in Tmux
sudo cp ~/robot_setup/deploy/robot_status.timer /etc/systemd/system/
sudo cp ~/robot_setup/deploy/robot_status.service /etc/systemd/system/

sudo cp ~/robot_setup/deploy/robot_hw_daemon.sh /etc/turtlebot4/robot_hw_daemon.sh
sudo chmod +x /etc/turtlebot4/robot_hw_daemon.sh

sudo systemctl daemon-reload
sudo systemctl enable --now robot_status.timer
sudo systemctl enable turtlebot_hw_daemon.service
sudo systemctl start turtlebot_hw_daemon.service

# Copies a sample wifi_config which has the AP mode config as well
# NOTE: the AP does not have a bot number. That would be generated using the "generate_configs.py"
# THis is required for changing to AP mode or changing Wifi using change_wifi.sh
cp wifi_configs_sample.json /home/ubuntu/wifi_configs.json


# Add "source ~/robot_setup/alias.zsh" to  bashrc if it does not exist
grep -qxF 'source ~/robot_setup/alias.zsh' ~/.bashrc || echo 'source ~/robot_setup/alias.zsh' >> ~/.bashrc

# change_wifi.py needs sudo priveleges to apply netplan and reboot computer
echo "=============================================="
echo "Add the following line to visudo:"
echo 'ubuntu ALL=(ALL) NOPASSWD: /usr/bin/python3 /home/ubuntu/robot_setup/change_wifi.py --ap --force'
echo "=============================================="
