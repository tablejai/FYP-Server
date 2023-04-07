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

command_list = ["STATIC", "SLIDE_UP", "SLIDE_DOWN", "SLIDE_LEFT",
                "SLIDE_RIGHT", "RELEASE", "GRASP", "HIGHLIGHT", "ON_YES", "OFF_NO", "NONE"]


class Commander(Node):

    def __init__(self):
        super().__init__('Commander')
        # Create a subscriber
        self.url = f"http://{LOCAL_IP}:{LOCAL_PORT}"
        self.led_url = f"http://{LED_IP}"
        self.gestures_subscriber = self.create_subscription(
            Gesture, '/Gestures', self.gesture_callback, 10)
        self.orientation_subscriber = self.create_subscription(
            Float32, '/Orientation', self.orientation_callback, 10)
        self.init_angle = None
        self.face_ppt = None

        command_state_init_time = datetime.now() - timedelta(seconds=1)
        self.command_states = {
            command: command_state_init_time for command in command_list}
        self.get_logger().info(f"aksdfss")


    def gesture_callback(self, msg):
        self.get_logger().info(f"command received: {msg.type}")
        self.get_logger().info(f"{self.face_ppt=}")
        if self.face_ppt is True:
            self.ppt_ctrl_flow(msg.type)
        elif self.face_ppt is False:
            self.led_ctrl_flow(msg.type)

    def orientation_callback(self, msg):
        # TODO: See how to get the proper angle
        # self.get_logger().info(f"{self.init_angle=}")
        # self.get_logger().info(f"{msg.data=}")
        angle = int(msg.data)
        if self.init_angle is None:
            self.init_angle = angle
            if self.init_angle >= 90 and self.init_angle <= 270:
                self.exceed_0 = False
                self.exceed_360 = False
                self.lower_bound = angle - 90
                self.upper_bound = angle + 90
            elif self.init_angle < 90:
                self.exceed_0 = True
                self.exceed_360 = False
                self.lower_bound = 270 + angle
                self.upper_bound = angle + 90
            elif self. init_angle > 270:
                self.exceed_0 = False
                self.exceed_360 = True
                self.lower_bound = angle - 90
                self.upper_bound = angle - 270
        else:
            if not self.exceed_0 and not self.exceed_360:
                self.face_ppt = angle > self.lower_bound and angle < self.upper_bound
            elif not self.exceed_0 and self.exceed_360:
                self.face_ppt = (angle > self.lower_bound and angle <
                                 360) or angle < self.lower_bound
            elif self.exceed_0 and not self.exceed_360:
                self.face_ppt = (angle < self.upper_bound and angle >
                                 0) or angle > self.upper_bound

    def ppt_ctrl_flow(self, command):
        print(self.command_states)
        cur_command = command_list[command]
        if datetime.now() - self.command_states[cur_command] > timedelta(seconds=2):
            json_to_sent = {"command": command}
            requests.post(self.url, json=json_to_sent)
            self.command_states[cur_command] = datetime.now()

    def led_ctrl_flow(self, command):
        print(self.command_states)
        cur_command = command_list[command]
        if datetime.now() - self.command_states[cur_command] > timedelta(seconds=1.3):
            if cur_command == "RELEASE" or cur_command == "GRASP":
                requests.get(
                    self.led_url, params="ON" if cur_command == "RELEASE" else "OFF")
            self.command_states[cur_command] = datetime.now()


def main(args=None):
    rclpy.init()
    node = Commander()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
