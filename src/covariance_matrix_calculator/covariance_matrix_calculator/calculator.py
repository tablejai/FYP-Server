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

BAG_ROOT_PATH = "/home/ubuntu/FYP-Glove/rosbag/bag/"

class Calculator(Node):
    def __init__(self, bag_name):
        super().__init__('Calculator')
        self.bag_path = BAG_ROOT_PATH + bag_name
        self.cal()

    def cal(self):
        # create reader instance and open for reading
        with Reader(self.bag_path) as reader:
            for connection in reader.connections:
                print(f"topic name: {connection.topic}\t count {connection.msgcount} msg type: {connection.msgtype}")

            imu_list = ["/Imu0", "/Imu1", "/Imu2"]
            ang_vels = {imu : [] for imu in imu_list}
            lin_accs = {imu : [] for imu in imu_list}

            # iterate over messages
            for connection, timestamp, rawdata in reader.messages():
                if connection.topic in imu_list:
                    imu = connection.topic
                    print(imu)

                    msg = deserialize_cdr(rawdata, connection.msgtype)
                    ang_vel = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
                    lin_acc = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
                    ang_vels[imu].append(ang_vel)
                    lin_accs[imu].append(lin_acc)
                    print(f"{ang_vel},\t{lin_acc}")

            for imu in imu_list:
                acc = np.array(lin_accs[imu])
                lin_cov = np.matmul(acc.T, acc) / (acc.shape[0] - 1)
                print(f"{imu} lin_cov=\n{lin_cov}")
                
                vel = np.array(ang_vels[imu])
                ang_cov = np.matmul(vel.T, vel) / (vel.shape[0] - 1)
                print(f"{imu} ang_cov=\n{ang_cov}")
                
                print("====================================")
                
def main(args=None):
    rclpy.init(args=args)
    
    bag_name = input(f"Please enter the name of the bag file: {BAG_ROOT_PATH}")
    while not os.path.exists(BAG_ROOT_PATH + bag_name):
        bag_name = input("File not exist, please enter the name of the bag file: ")
    
    calculator = Calculator(bag_name)

    calculator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
