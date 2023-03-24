import rclpy
from rclpy.node import Node

from datetime import datetime, timedelta
from std_msgs.msg import String, Float32
from msgs.msg import Gesture
import requests
import json
LOCAL_IP = "192.168.4.2"
LOCAL_PORT = "7000"

command_list = ["STATIC", "SLIDE_UP", "SLIDE_DOWN", "SLIDE_LEFT", "SLIDE_RIGHT", "RELEASE", "GRASP", "HIGHLIGHT","ON_YES","OFF_NO","NONE"]

class Commander(Node):

    def __init__(self):
        super().__init__('Commander')
        # Create a subscriber
        self.url = f"http://{LOCAL_IP}:{LOCAL_PORT}"
        self.gestures_subscriber = self.create_subscription(Gesture, '/Gestures', self.gesture_callback, 10)
        self.orientation_subscriber = self.create_subscription(Float32, '/Orientation', self.orientation_callback, 10)

        command_state_init_time = datetime.now() - timedelta(seconds= 1)
        self.command_states = {command: command_state_init_time for command in command_list}
    
    def gesture_callback(self, msg):
        print(f"command received: {msg.type}")
        self.command_ctrl_flow(msg.type)
    
    def orientation_callback(self, msg):
        self.get_logger().info(f"command received: {msg.data}")
        
    def command_ctrl_flow(self, command):
        print(self.command_states)
        cur_command = command_list[command]
        if datetime.now() - self.command_states[cur_command] > timedelta(seconds=2):
            json_to_sent = {"command": command}
            requests.post(self.url, json=json_to_sent)
            self.command_states[cur_command] = datetime.now()
        
def main(args=None):
    rclpy.init()
    node = Commander()
    rclpy.spin(node)

    node.destroy_node() 
    rclpy.shutdown()

if __name__ == '__main__':
    main()
