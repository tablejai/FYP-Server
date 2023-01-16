import rclpy
from rclpy.node import Node

from pathlib import Path
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr

# math
import numpy as np
import math

# threading
import threading

# files
import os

BAG_ROOT_PATH = "/home/ubuntu/FYP-ROS/rosbag/bag/"

class Calculator(Node):
    def __init__(self, bag_name, topic):
        super().__init__('Calculator')
        self.bag_path = BAG_ROOT_PATH + bag_name
        self.topic = topic
        self.cal()

    def cal(self):
        # create reader instance and open for reading
        with Reader(self.bag_path) as reader:
            for connection in reader.connections:
                print(f"topic name: {connection.topic}\t count {connection.msgcount} msg type: {connection.msgtype}")

            # iterate over messages
            ang_vels = []
            lin_accs = []
            for connection, timestamp, rawdata in reader.messages():
                if connection.topic == self.topic:
                    msg = deserialize_cdr(rawdata, connection.msgtype)
                    ang_vel = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
                    lin_acc = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
                    ang_vels.append(ang_vel)
                    lin_accs.append(lin_acc)
                    print(f"{ang_vel},\t{lin_acc}")

            ang_cov = np.cov(ang_vels, rowvar=False)
            lin_cov = np.cov(lin_accs, rowvar=False)
            print(f"{ang_cov=}")
            print(f"{lin_cov=}")

def main(args=None):
    rclpy.init(args=args)
    
    bag_name = input(f"Please enter the name of the bag file: {BAG_ROOT_PATH}")
    while not os.path.exists(BAG_ROOT_PATH + bag_name):
        bag_name = input("File not exist, please enter the name of the bag file: ")
    
    topic_name = input("Please enter the topic name: ")

    calculator = Calculator(bag_name, topic_name)

    calculator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
