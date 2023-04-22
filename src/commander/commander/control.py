import rclpy
from rclpy.node import Node

from datetime import datetime, timedelta
from std_msgs.msg import String, Float32
from msgs.msg import Gesture
import requests
import json

LOCAL_IP = "host.docker.internal"
LED_IP = "192.168.4.3"
LOCAL_PORT = "7000"
dev_config_list = []

command_list = [
    "STATIC",
    "SLIDE_UP",
    "SLIDE_DOWN",
    "SLIDE_LEFT",
    "SLIDE_RIGHT",
    "RELEASE",
    "GRASP",
    "HIGHLIGHT",
    "ON_YES",
    "OFF_NO",
    "NONE",
]


class Commander(Node):
    def __init__(self):
        super().__init__("Commander")
        # Create a subscriber
        self.load_dev_config()
        self.url = "http://{}:7000"
        self.led_url = "http://{}"
        self.gestures_subscriber = self.create_subscription(
            Gesture, "/Gestures", self.gesture_callback, 10
        )
        self.orientation_subscriber = self.create_subscription(
            Float32, "/Orientation", self.orientation_callback, 10
        )
        self.init_angle = None
        self.face_ppt = None
        self.send_ip = "192.168.4.1"

        command_state_init_time = datetime.now() - timedelta(seconds=1)
        self.command_states = {
            command: command_state_init_time for command in command_list
        }
        self.get_logger().info(f"aksdfss")

    def load_dev_config(self):
        global dev_config_list
        with open(
            "/home/ubuntu/FYP-Glove/src/commander/commander/dev_config.json"
        ) as dev_config_file:
            dev_dict = json.load(dev_config_file)
            for dev_item in dev_dict.values():
                u_bound = dev_item["upper_bound"]
                l_bound = dev_item["lower_bound"]
                dev_type = dev_item["device_type"]
                dev_config_obj = {
                    "lower_bound": float(l_bound),
                    "upper_bound": float(u_bound),
                    "dev_type": dev_type,
                    "exceed_0": True if float(l_bound) > float(u_bound) else False,
                    "exceed_360": True if float(l_bound) < float(u_bound) else False,
                    "ip_address": dev_item["ip_address"],
                }
                dev_config_list.append(dev_config_obj)

    def gesture_callback(self, msg):
        if self.face_ppt is True:
            self.ppt_ctrl_flow(msg.type)
        elif self.face_ppt is False:
            self.led_ctrl_flow(msg.type)

    def orientation_callback(self, msg):
        # TODO: See how to get the proper angle
        send_dev = None
        send_ip = None
        global dev_config_list
        angle = int(msg.data)
        for dev in dev_config_list:
            lower_bound = dev["lower_bound"]
            upper_bound = dev["upper_bound"]
            exceed_0 = dev["exceed_0"]
            exceed_360 = dev["exceed_360"]
            if not exceed_0 and not exceed_360:
                if angle >= lower_bound and angle >= upper_bound:
                    send_dev = dev["dev_type"]
                    send_ip = dev["ip_address"]
                    break
            elif not exceed_0 and exceed_360:
                if (angle >= lower_bound and angle <= 360) or angle <= lower_bound:
                    send_dev = dev["dev_type"]
                    send_ip = dev["ip_address"]
                    break
            elif exceed_0 and not exceed_360:
                if (angle <= upper_bound and angle >= 0) or angle >= upper_bound:
                    send_dev = dev["dev_type"]
                    send_ip = dev["ip_address"]
                    break
        self.get_logger().info(f"send_dev: {send_dev}")
        self.get_logger().info(f"device_ip: {send_ip}")
        if send_dev == "Powerpoint":
            self.face_ppt = True
            self.send_ip = send_ip
        else:
            self.face_ppt = False
            self.send_ip = send_ip

    def ppt_ctrl_flow(self, command):
        cur_command = command_list[command]
        if datetime.now() - self.command_states[cur_command] > timedelta(seconds=2):
            json_to_sent = {"command": command}
            requests.post(self.url.format(self.send_ip), json=json_to_sent)
            self.command_states[cur_command] = datetime.now()

    def led_ctrl_flow(self, command):
        cur_command = command_list[command]
        if datetime.now() - self.command_states[cur_command] > timedelta(seconds=1.3):
            if cur_command == "RELEASE" or cur_command == "GRASP":
                requests.get(
                    self.led_url.format(self.send_ip),
                    params="ON" if cur_command == "RELEASE" else "OFF",
                )
            self.command_states[cur_command] = datetime.now()


def main(args=None):
    rclpy.init()
    node = Commander()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
