import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from msgs.msg import Geasture
import requests
import json

LOCAL_IP = "192.168.0.104"
LOCAL_PORT = "7000"

class Commander(Node):

    def __init__(self):
        super().__init__('Commander')

        # Create a subscriber
        self.subscriber = self.create_subscription(Geasture, '/Geastures', self.callback, 10)
    
    def callback(self, msg):
        self.send_request(msg.type)

    def send_request(self, msg):
        send_json = {"command": msg}
        url = f"http://{LOCAL_IP}:{LOCAL_PORT}"
        requests.post(url, json=test_obj)
        
def main(args=None):
    rclpy.init()
    node = Commander()
    rclpy.spin(node)

    node.destroy_node() 
    rclpy.shutdown()

if __name__ == '__main__':
    main()
