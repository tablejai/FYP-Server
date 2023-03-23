import rclpy
from rclpy.node import Node

from datetime import datetime, timedelta
from std_msgs.msg import String
from msgs.msg import Geasture
import requests
import json
LOCAL_IP = "10.89.10.144"
LOCAL_PORT = "7000"

command_list = ["STATIC", "SLIDE_UP", "SLIDE_DOWN", "SLIDE_LEFT", "SLIDE_RIGHT", "RELEASE", "GRASP", "HIGHLIGHT","ON_YES","OFF_NO","NONE"]

class Commander(Node):

    def __init__(self):
        super().__init__('Commander')
        # Create a subscriber
        self.url = f"http://{LOCAL_IP}:{LOCAL_PORT}"
        self.subscriber = self.create_subscription(Geasture, '/Geastures', self.callback, 10)
        command_state_init_time = datetime.now() - timedelta(seconds= 1)
        self.command_states = {command: command_state_init_time for command in command_list}
    
    def callback(self, msg):
        print(f"command received: {msg.type}")
        self.command_ctrl_flow(msg.type)

    def command_ctrl_flow(self, command):
        print(self.command_states)
        cur_command = command_list[command]
        if datetime.now() - self.command_states[cur_command] > timedelta(seconds=1):
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
