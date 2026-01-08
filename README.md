# 1. Setup Config

## Backup and Replace bashrc
```bash
cp ~/.bashrc ~/.bashrc_$(date +%Y%m%d_%H%M%S) 
cp bashrc ~/.bashrc
```

## Backup and Replace tmux conf
```bash
cp ~/.tmux.conf ~/.tmux_$(date +%Y%m%d_%H%M%S) 
cp tmux.conf ~/.tmux.conf
```

## Backup and Replace vimrc
```bash
cp ~/.vimrc ~/.vimrc_$(date +%Y%m%d_%H%M%S) 
cp vimrc ~/.vimrc
```

# 2. TurtleBot 4 Status Monitor for Tmux

This setup allows your `tmux` status bar to display the TurtleBot 4 battery percentage and docking status using Nerd Font icons. It uses a **systemd timer** to fetch data in the background every minute, ensuring the tmux interface never lags.

## 1. Prerequisites

Ensure you have the following installed on your TurtleBot 4:
* **bc**: For battery calculations (`sudo apt install bc`)
* **Nerd Fonts**: Your terminal must use a Nerd Font (e.g., Meslo, JetBrains Mono) to display icons correctly.

## 2. Create Systemd Service

```bash
chmod +x ~/robot_setup/fetch_robot_stats.sh
```

Create a service unit file to define the background execution.

**File Path:** `/etc/systemd/system/robot_status.service`

```ini
[Unit]
Description=Fetch Robot Status

[Service]
Type=oneshot
User=ubuntu
ExecStart=/bin/bash /home/ubuntu/robot_setup/fetch_robot_stats.sh

```

## 3. Create Systemd Timer

Create a timer unit file to trigger the service every minute.

**File Path:** `/etc/systemd/system/robot_status.timer`

```ini
[Unit]
Description=Run Robot Status Fetcher Every 20 Seconds

[Timer]
# Initial delay after boot
OnBootSec=1min
# Time between consecutive runs
OnUnitActiveSec=20s
# Ensures the timer stays accurate
AccuracySec=1s

[Install]
WantedBy=timers.target

```

## 4. Enable and Start

Reload the systemd manager configuration, then enable and start the timer.

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now robot_status.timer

```

## 5. Configure Tmux

Update your `~/.tmux.conf` to display the status data from the temporary file.

```tmux
# Refresh interval for the status bar
set -g status-interval 5

# Status bar right-side configuration
set -g status-right-length 90
set -g status-right "#(cat /tmp/robot_status 2>/dev/null || echo 'Connecting...') | %H:%M "

```

Reload the configuration within tmux:
`tmux source-file ~/.tmux.conf`

## 6. Debugging

Use these commands to verify that the background process is running correctly:

```bash
# Check timer schedule and status
systemctl status robot_status.timer

# Check logs for the service execution
journalctl -u robot_status.service

```
