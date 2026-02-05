#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import subprocess
import sys
from sensor_msgs.msg import BatteryState
from irobot_create_msgs.msg import LightringLeds, LedColor, AudioNoteVector, AudioNote, InterfaceButtons
from builtin_interfaces.msg import Duration
import os

class RobotController(Node):
    def __init__(self):
        # Get robot namespace from stored file
        robot_id = self.get_robot_id()
        namespace = f"robot_{robot_id}"
        super().__init__('robot_hw_daemon', namespace=namespace)

        # --- Publishers ---
        self.lightring_pub = self.create_publisher(LightringLeds, 'cmd_lightring', 10)
        self.audio_pub = self.create_publisher(AudioNoteVector, 'cmd_audio', 10)

        # --- Subscriptions ---
        self.battery_sub = self.create_subscription(
            BatteryState, 'battery_state', self.battery_callback, 10)
        
        self.button_sub = self.create_subscription(
            InterfaceButtons, 'interface_buttons', self.button_callback, 10)

        # --- State Variables ---
        self.startup_done = False
        self.press_count = 0
        self.last_button_state = False
        self.last_press_time = 0.0
        
        # --- Battery Alert Variables ---
        self.current_battery_percentage = 1.0 
        self.is_charging = False               # Tracks if robot is on dock
        self.battery_threshold = 0.20          
        
        # --- Config ---
        self.python_executable = sys.executable
        self.target_script = "/home/ubuntu/robot_setup/change_wifi.py"
        self.target_args = "--ap --force"
        
        self.required_presses = 5
        self.button_timeout = 3.0 

        # --- Timers ---
        # Check battery level every 5 minutes (300 seconds)
        self.battery_alert_timer = self.create_timer(300.0, self.check_battery_alert)

        self.get_logger().info(f'Jazzy Manager [{namespace}] Active: Waiting for heartbeat...')
    
    def get_robot_id(self):
        id_file = "/home/ubuntu/.turtlebot_id"
        if os.path.exists(id_file):
            with open(id_file, 'r') as f:
                return f.read().strip()
        return "XX"

    def battery_callback(self, msg):
        """Updates battery state and tracks charging status."""
        self.current_battery_percentage = msg.percentage
        
        # Status 1 = Charging, Status 4 = Full (both mean it is on the dock)
        # Status 2 = Discharging, Status 3 = Not Charging (meaning it's off dock)
        self.is_charging = msg.power_supply_status in [BatteryState.POWER_SUPPLY_STATUS_CHARGING, 
                                                       BatteryState.POWER_SUPPLY_STATUS_FULL]

        if not self.startup_done:
            self.startup_done = True
            self.get_logger().info('Communication established! Playing chime.')
            self.play_startup_sound()
            self.run_circle_animation(color="pink", duration_sec=4.0)

    def check_battery_alert(self):
        """Timer callback to warn if battery is low, but only if NOT charging."""
        if self.is_charging:
            # self.get_logger().info('Battery low but charging/docked. Muting alert.')
            return

        if self.current_battery_percentage <= self.battery_threshold:
            self.get_logger().warn(f'LOW BATTERY ALERT: {self.current_battery_percentage*100:.1f}%')
            self.play_low_battery_sound()
            self.run_circle_animation(color="red", duration_sec=2.0)

    def button_callback(self, msg):
        """Monitors for 5 presses on Button 1."""
        current_state = msg.button_1.is_pressed

        if current_state and not self.last_button_state:
            now = time.time()
            
            if now - self.last_press_time > self.button_timeout:
                self.press_count = 1
                self.get_logger().info('Button sequence started (1/5)')
            else:
                self.press_count += 1
                self.get_logger().info(f'Presses: {self.press_count}/{self.required_presses}')
            
            self.last_press_time = now

            if self.press_count == self.required_presses:
                self.get_logger().info('5 presses detected! Triggering WiFi script.')
                self.play_long_beep()
                self.run_circle_animation(color="green", duration_sec=5.0)
                self.execute_wifi_script()
                self.press_count = 0 

        self.last_button_state = current_state

    def play_startup_sound(self):
        if not self.wait_for_audio_subscriber():
            return

        audio_msg = AudioNoteVector()
        audio_msg.header.stamp = self.get_clock().now().to_msg()
        notes = [
            AudioNote(frequency=659, max_runtime=Duration(sec=0, nanosec=150000000)),
            AudioNote(frequency=784, max_runtime=Duration(sec=0, nanosec=150000000)),
            AudioNote(frequency=880, max_runtime=Duration(sec=0, nanosec=150000000)),
            AudioNote(frequency=784, max_runtime=Duration(sec=0, nanosec=300000000)),
        ]
        audio_msg.notes = notes
        self.audio_pub.publish(audio_msg)

    def play_low_battery_sound(self):
        audio_msg = AudioNoteVector()
        audio_msg.header.stamp = self.get_clock().now().to_msg()
        notes = [
            AudioNote(frequency=440, max_runtime=Duration(sec=0, nanosec=200000000)),
            AudioNote(frequency=330, max_runtime=Duration(sec=0, nanosec=400000000)),
        ]
        audio_msg.notes = notes
        self.audio_pub.publish(audio_msg)

    def play_long_beep(self):
        audio_msg = AudioNoteVector()
        audio_msg.header.stamp = self.get_clock().now().to_msg()
        long_beep = AudioNote(frequency=500, max_runtime=Duration(sec=2, nanosec=0))
        audio_msg.notes = [long_beep]
        self.audio_pub.publish(audio_msg)

    def wait_for_audio_subscriber(self):
        attempts = 0
        while self.audio_pub.get_subscription_count() == 0 and attempts < 20:
            time.sleep(0.5)
            attempts += 1
        return self.audio_pub.get_subscription_count() > 0

    def reset_leds(self):
        msg = LightringLeds()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.override_system = False
        self.lightring_pub.publish(msg)

    def run_circle_animation(self, color, duration_sec):
        start_time = time.time()
        led_index = 0
        
        if color == "pink":
            target_color = LedColor(red=255, green=20, blue=147)
        elif color == "green":
            target_color = LedColor(red=0, green=255, blue=0)
        else: # red
            target_color = LedColor(red=255, green=0, blue=0)
            
        off = LedColor(red=0, green=0, blue=0)

        while (time.time() - start_time) < duration_sec:
            msg = LightringLeds(override_system=True)
            msg.header.stamp = self.get_clock().now().to_msg()
            leds = [off] * 6
            leds[led_index] = target_color
            msg.leds = leds
            self.lightring_pub.publish(msg)
            led_index = (led_index + 1) % 6
            time.sleep(0.06)
        
        self.reset_leds()

    def execute_wifi_script(self):
        try:
            cmd = f"sudo {self.python_executable} {self.target_script} {self.target_args}"
            self.get_logger().info(f'Launching WiFi script: {cmd}')
            subprocess.Popen(cmd, shell=True)
        except Exception as e:
            self.get_logger().error(f'Failed to launch WiFi script: {e}')
    
def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()