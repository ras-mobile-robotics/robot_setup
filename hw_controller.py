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

# Terminal Colors for Robot Output
class TermCol:
    OK = '\033[92m'
    INFO = '\033[94m'
    WARN = '\033[93m'
    FAIL = '\033[91m'
    BOLD = '\033[1m'
    END = '\033[0m'

class RobotHWDaemon(Node):
    def __init__(self):
        # 1. Setup Logging
        self.log_file = os.path.expanduser("~/.last_shutdown_log")
        self.setup_logger()

        # 2. Get Namespace
        self.robot_id = self.get_robot_id()
        self.namespace = f"robot_{self.robot_id}"
        
        super().__init__('robot_hw_daemon', namespace=self.namespace)
        print(f"{TermCol.BOLD}{TermCol.INFO}[INIT]{TermCol.END} Controller Active: {TermCol.BOLD}{self.namespace}{TermCol.END}")

<<<<<<< HEAD
        # 3. Setup Clients
        self.undock_client = ActionClient(self, Undock, f'/{self.namespace}/undock')
        self.dock_client = ActionClient(self, Dock, f'/{self.namespace}/dock')
        self.power_client = self.create_client(RobotPower, f'/{self.namespace}/robot_power')
=======
        # 3. Topic/Service Names with explicit Namespacing
        self.undock_action_name = f'/{self.namespace}/undock'
        self.dock_action_name = f'/{self.namespace}/dock'
        self.power_service_name = f'/{self.namespace}/robot_power'
        self.batt_topic = f'/{self.namespace}/battery_state'

        # 4. Setup Clients
        print(f"{TermCol.INFO}[TOPIC]{TermCol.END} Tracking {self.batt_topic}")
        self.undock_client = ActionClient(self, Undock, self.undock_action_name)
        self.dock_client = ActionClient(self, Dock, self.dock_action_name)
        self.power_client = self.create_client(RobotPower, self.power_service_name)
>>>>>>> 95530d1 (update namespace)
        
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

    def generate_report(self):
        """Gathers stats and prints as JSON"""
        # Spin to catch battery message
        timeout = time.time() + 3.0
        while self.latest_battery is None and time.time() < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        # Get System Stats
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                temp = int(f.read()) / 1000.0
        except: temp = 0.0
        
        uptime = subprocess.check_output(["uptime", "-p"]).decode().strip()
        disk = subprocess.check_output(["df", "-h", "/"]).decode().split('\n')[1].split()[4]

        report = {
            "robot_id": self.robot_id,
            "battery_pct": round(self.latest_battery, 1) if self.latest_battery is not None else "N/A",
            "cpu_temp_c": temp,
            "uptime": uptime,
            "disk_usage": disk,
            "timestamp": time.strftime("%H:%M:%S")
        }
        print(f"REPORT_DATA:{json.dumps(report)}")

    def step_undock(self):
        print(f"{TermCol.INFO}[ACTION]{TermCol.END} Calling Undock on {self.undock_action_name}")
        if not self.undock_client.wait_for_server(timeout_sec=5.0):
            return False
        goal_msg = Undock.Goal()
        future = self.undock_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        return True

    def step_sync(self):
        print(f"{TermCol.INFO}[ACTION]{TermCol.END} Syncing SD Card...")
        subprocess.run(["sync"], check=True)
        return True

    def step_base_power(self):
        print(f"{TermCol.FAIL}[POWER]{TermCol.END} Killing base power: {self.power_service_name}")
        if self.power_client.wait_for_service(timeout_sec=5.0):
            req = RobotPower.Request()
            self.power_client.call_async(req)
            return True
        return False

    def full_shutdown_sequence(self):
        print(f"{TermCol.WARN}[SHUTDOWN]{TermCol.END} Initializing Safe-Idle...")


        self.generate_report() # Log stats one last time

        # Undock robot
        self.step_undock()
        time.sleep(7)

        # Double sync
        self.step_sync()
        time.sleep(1)
        self.step_sync() 

        # Kill Power
        self.step_base_power()

def main(args=None):
    rclpy.init(args=args)
    daemon = RobotHWDaemon()
    
    if len(sys.argv) > 1:
        command = sys.argv[1].lower()
        if command == "shutdown":
            daemon.full_shutdown_sequence()
        elif command == "report":
            daemon.generate_report()
        else:
            print(f"Unknown command: {command}")
    
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()