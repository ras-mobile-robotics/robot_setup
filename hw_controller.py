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

class RobotHWDaemon(Node):
    def __init__(self):
        # 1. Setup Logging
        self.log_file = os.path.expanduser("~/.last_shutdown_log")
        self.setup_logger()

        # 2. Get Namespace
        self.robot_id = self.get_robot_id()
        self.namespace = f"robot_{self.robot_id}"
        
        super().__init__('robot_hw_daemon', namespace=self.namespace)
        self.logger.info(f"--- Controller Active: {self.namespace} ---")

        # 3. Setup Clients
        self.undock_client = ActionClient(self, Undock, f'/{self.namespace}/undock')
        self.dock_client = ActionClient(self, Dock, f'/{self.namespace}/dock')
        self.power_client = self.create_client(RobotPower, f'/{self.namespace}/robot_power')
        
        # Internal storage for report
        self.latest_battery = None
        self.batt_sub = self.create_subscription(
            BatteryState, f'/{self.namespace}/battery_state', self._batt_cb, 10)

    def _batt_cb(self, msg):
        self.latest_battery = msg.percentage * 100

    def setup_logger(self):
        self.logger = logging.getLogger("RobotHW")
        self.logger.setLevel(logging.INFO)
        f_handler = logging.FileHandler(self.log_file, mode='w')
        c_handler = logging.StreamHandler()
        formatter = logging.Formatter('[%(asctime)s] %(levelname)s: %(message)s', datefmt='%Y-%m-%d %H:%M:%S')
        f_handler.setFormatter(formatter)
        c_handler.setFormatter(formatter)
        self.logger.addHandler(f_handler)
        self.logger.addHandler(c_handler)

    def get_robot_id(self):
        id_file = "/home/ubuntu/.turtlebot_id"
        if os.path.exists(id_file):
            with open(id_file, 'r') as f:
                return f.read().strip()
        return "XX"

    def generate_report(self):
        """Gathers stats and prints as JSON for the collector script"""
        # Get Battery (Spin a bit to catch the subscription)
        timeout = time.time() + 2.0
        while self.latest_battery is None and time.time() < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        # Get CPU Temp
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                temp = int(f.read()) / 1000.0
        except: temp = 0.0

        # Get Uptime
        uptime = subprocess.check_output(["uptime", "-p"]).decode().strip()

        # Get Disk Usage
        disk = subprocess.check_output(["df", "-h", "/"]).decode().split('\n')[1].split()[4]

        report = {
            "robot_id": self.robot_id,
            "battery_pct": round(self.latest_battery, 1) if self.latest_battery else "N/A",
            "cpu_temp_c": temp,
            "uptime": uptime,
            "disk_usage": disk,
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")
        }
        # Print with a unique prefix so the collector can find it easily
        print(f"REPORT_DATA:{json.dumps(report)}")

    # --- INDIVIDUAL STEP FUNCTIONS ---

    def step_undock(self):
        self.logger.info("ACTION: Undocking from base...")
        if not self.undock_client.wait_for_server(timeout_sec=5.0):
            self.logger.error("Undock server not found.")
            return False
        goal_msg = Undock.Goal()
        future = self.undock_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        self.logger.info("Undock command successful.")
        return True

    def step_dock(self):
        self.logger.info("ACTION: Docking to base...")
        if not self.dock_client.wait_for_server(timeout_sec=5.0):
            self.logger.error("Dock server not found.")
            return False
        goal_msg = Dock.Goal()
        future = self.dock_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        return True

    def step_sync(self):
        self.logger.info("ACTION: Syncing SD card buffers...")
        try:
            subprocess.run(["sync"], check=True)
            self.logger.info("Sync complete.")
            return True
        except Exception as e:
            self.logger.error(f"Sync failed: {e}")
            return False

    def step_base_power(self):
        self.logger.info(f"ACTION: Calling {self.namespace}/robot_power service...")
        if self.power_client.wait_for_service(timeout_sec=5.0):
            req = RobotPower.Request()
            self.power_client.call_async(req)
            self.logger.info("Power-off request sent to Create 3.")
            return True
        else:
            self.logger.error("RobotPower service timed out!")
            return False

    def step_pi_halt(self):
        self.logger.warning("ACTION: Halting Raspberry Pi OS...")
        subprocess.run(["sudo", "shutdown", "-h", "now"])

    def full_shutdown_sequence(self):
        self.logger.info("Starting Safe-Idle Shutdown...")
        self.step_undock()
        time.sleep(7) 
        self.logger.info("Entering Quiet State...")
        self.step_sync()
        time.sleep(1)
        self.step_sync() 
        self.logger.info("SD Card is now 'Cold' (Safe for power cut)")
        self.logger.info("Power cut starting...")
        self.step_base_power()
        self.logger.info("Power cut imminent. Standby.")

def main(args=None):
    rclpy.init(args=args)
    daemon = RobotHWDaemon()
    
    if len(sys.argv) > 1:
        command = sys.argv[1].lower()
        if command == "shutdown":
            daemon.full_shutdown_sequence()
        elif command == "report":
            daemon.generate_report()
        elif command == "undock":
            daemon.step_undock()
        elif command == "dock":
            daemon.step_dock()
        elif command == "sync":
            daemon.step_sync()
        elif command == "power_base":
            daemon.step_base_power()
        elif command == "halt_pi":
            daemon.step_pi_halt()
        else:
            print(f"Unknown command: {command}")
    else:
        print("Usage: python3 hw_controller.py [shutdown|report|undock|dock|sync|power_base|halt_pi]")

    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()