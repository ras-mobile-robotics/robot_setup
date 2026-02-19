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
        # 1. Setup Logging and Start Timer
        self.start_time = time.time()
        self.log_file = os.path.expanduser("~/.last_shutdown_log")
        self.runtime_log = os.path.expanduser("~/robot_setup/total_runtime.log")
        self.setup_logger()

        self.robot_id = self.get_robot_id()
        self.namespace = f"robot_{self.robot_id}"
        
        super().__init__('robot_hw_daemon', namespace=self.namespace)
        print(f"{TermCol.BOLD}{TermCol.INFO}[INIT]{TermCol.END} Controller Active: {self.namespace}")

        # 2. Topic/Service Names
        self.undock_action_name = f'/{self.namespace}/undock'
        self.dock_action_name = f'/{self.namespace}/dock'
        self.power_service_name = f'/{self.namespace}/robot_power'
        self.batt_topic = f'/{self.namespace}/battery_state'

        # 3. Setup Clients
        self.undock_client = ActionClient(self, Undock, self.undock_action_name)
        self.dock_client = ActionClient(self, Dock, self.dock_action_name)
        self.power_client = self.create_client(RobotPower, self.power_service_name)
        
        self.latest_battery = None
        self.batt_sub = self.create_subscription(BatteryState, self.batt_topic, self._batt_cb, 10)

    def _batt_cb(self, msg):
        self.latest_battery = msg.percentage * 100

    def setup_logger(self):
        self.logger = logging.getLogger("RobotHW")
        self.logger.setLevel(logging.INFO)
        f_handler = logging.FileHandler(self.log_file, mode='w')
        formatter = logging.Formatter('[%(asctime)s] %(levelname)s: %(message)s', datefmt='%Y-%m-%d %H:%M:%S')
        f_handler.setFormatter(formatter)
        self.logger.addHandler(f_handler)

    def get_robot_id(self):
        id_file = os.path.expanduser("~/.turtlebot_id")
        if os.path.exists(id_file):
            with open(id_file, 'r') as f:
                return f.read().strip()
        return "XX"

    def log_runtime(self):
        """Calculates and saves the duration of this session."""
        duration = time.time() - self.start_time
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        with open(self.runtime_log, "a") as f:
            f.write(f"{timestamp} | Duration: {round(duration, 2)} seconds\n")

    def generate_report(self):
        timeout = time.time() + 3.0
        while self.latest_battery is None and time.time() < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        # Calculate "Total Accumulated Hours" from the log file
        total_seconds = 0
        if os.path.exists(self.runtime_log):
            with open(self.runtime_log, "r") as f:
                for line in f:
                    if "Duration:" in line:
                        try:
                            total_seconds += float(line.split("Duration:")[1].split()[0])
                        except: pass

        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                temp = int(f.read()) / 1000.0
        except: temp = 0.0
        
        report = {
            "robot_id": self.robot_id,
            "battery_pct": round(self.latest_battery, 1) if self.latest_battery is not None else "N/A",
            "cpu_temp_c": temp,
            "session_runtime_sec": round(time.time() - self.start_time, 2),
            "weekly_total_hrs": round(total_seconds / 3600.0, 2),
            "timestamp": time.strftime("%H:%M:%S")
        }
        print(f"REPORT_DATA:{json.dumps(report)}")

    def full_shutdown_sequence(self):
        print(f"{TermCol.WARN}[SHUTDOWN]{TermCol.END} Logging runtime and powering off...")
        self.log_runtime() # Save the hours before the power is cut
        
        # Hardware sequence
        if not self.undock_client.wait_for_server(timeout_sec=5.0):
            print(f"{TermCol.FAIL}[ERROR]{TermCol.END} Undock server offline.")
        else:
            goal_msg = Undock.Goal()
            future = self.undock_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, future)
            time.sleep(7) 

        subprocess.run(["sync"], check=True)
        if self.power_client.wait_for_service(timeout_sec=5.0):
            req = RobotPower.Request()
            self.power_client.call_async(req)
        
def main(args=None):
    rclpy.init(args=args)
    daemon = RobotHWDaemon()
    
    if len(sys.argv) > 1:
        command = sys.argv[1].lower()
        if command == "shutdown":
            daemon.full_shutdown_sequence()
        elif command == "report":
            daemon.generate_report()
    
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()