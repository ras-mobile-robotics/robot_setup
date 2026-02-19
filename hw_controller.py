import os
import time
import subprocess
import logging
import rclpy
import sys
import json
from rclpy.node import Node
from rclpy.action import ActionClient

# Message Types
from irobot_create_msgs.action import Dock, Undock
from irobot_create_msgs.srv import RobotPower
from sensor_msgs.msg import BatteryState

class TermCol:
    OK = '\033[92m'
    INFO = '\033[94m'
    WARN = '\033[93m'
    FAIL = '\033[91m'
    BOLD = '\033[1m'
    END = '\033[0m'

class RobotHWDaemon(Node):
    def __init__(self):
        self.start_time = time.time()
        self.runtime_log = os.path.expanduser("~/total_runtime.log")
        
        # Charging state tracking
        self.last_state_change = time.time()
        self.is_currently_charging = False
        
        self.robot_id = self.get_robot_id()
        self.namespace = f"robot_{self.robot_id}"
        
        super().__init__('robot_hw_daemon', namespace=self.namespace)
        
        # Topic/Service Names
        self.undock_action_name = f'/{self.namespace}/undock'
        self.power_service_name = f'/{self.namespace}/robot_power'
        self.batt_topic = f'/{self.namespace}/battery_state'

        self.undock_client = ActionClient(self, Undock, self.undock_action_name)
        self.power_client = self.create_client(RobotPower, self.power_service_name)
        
        self.latest_battery = None
        self.batt_sub = self.create_subscription(BatteryState, self.batt_topic, self._batt_cb, 10)

    def _batt_cb(self, msg):
        self.latest_battery = msg.percentage * 100
        
        # Check power_supply_status: 1 = Charging
        charging_now = (msg.power_supply_status == 1)
        
        if charging_now != self.is_currently_charging:
            self.log_session_segment(self.is_currently_charging)
            self.is_currently_charging = charging_now
            self.last_state_change = time.time()

    def get_robot_id(self):
        id_file = os.path.expanduser("~/.turtlebot_id")
        if os.path.exists(id_file):
            with open(id_file, 'r') as f: return f.read().strip()
        return "XX"

    def log_session_segment(self, was_charging):
        """Logs a segment of time to the persistent log."""
        duration = time.time() - self.last_state_change
        label = "CHARGING" if was_charging else "IN_USE"
        with open(self.runtime_log, "a") as f:
            f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} | {label} | {round(duration, 2)}\n")

    def generate_report(self):
        # Update current segment before reporting
        self.log_session_segment(self.is_currently_charging)
        self.last_state_change = time.time()

        total_use = 0.0
        total_charge = 0.0
        
        if os.path.exists(self.runtime_log):
            with open(self.runtime_log, "r") as f:
                for line in f:
                    parts = line.split("|")
                    if len(parts) < 3: continue
                    mode = parts[1].strip()
                    val = float(parts[2].strip())
                    if mode == "IN_USE": total_use += val
                    elif mode == "CHARGING": total_charge += val

        report = {
            "robot_id": self.robot_id,
            "battery_pct": round(self.latest_battery, 1) if self.latest_battery is not None else "N/A",
            "in_use_hrs": round(total_use / 3600.0, 2),
            "charging_hrs": round(total_charge / 3600.0, 2),
            "is_charging": self.is_currently_charging,
            "timestamp": time.strftime("%H:%M:%S")
        }
        print(f"REPORT_DATA:{json.dumps(report)}")

    def full_shutdown_sequence(self):
        self.log_session_segment(self.is_currently_charging)
        # ... (rest of undock and power logic same as before) ...
        subprocess.run(["sync"], check=True)
        if self.power_client.wait_for_service(timeout_sec=5.0):
            req = RobotPower.Request()
            self.power_client.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    daemon = RobotHWDaemon()
    if len(sys.argv) > 1:
        command = sys.argv[1].lower()
        if command == "shutdown": daemon.full_shutdown_sequence()
        elif command == "report": daemon.generate_report()
    if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()