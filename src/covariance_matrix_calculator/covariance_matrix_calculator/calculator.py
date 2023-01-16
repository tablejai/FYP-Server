import rclpy
from rclpy.node import Node

from pathlib import Path
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr

# threading
import threading

# files
import os

class Calculator(Node):

    def __init__(self, bag_name):
        super().__init__('Calculator')
        self.bag_path = '/home/ubuntu/FYP-ROS/rosbag/bag/' + bag_name
        
        self.label_thread = threading.Thread(target=self.start_labeling)
        self.label_thread.start()

    def start_labeling(self):
        ttl_raw_count = 0
        # create reader instance and open for reading
        with Reader(self.bag_path) as reader:
            # topic and msgtype information is available on .connections list
            for connection in reader.connections:
                print("topic name: {0: <15}\t count {1: <15} msg type: {2: <15}".format(connection.topic, connection.msgcount, connection.msgtype))
            print("\n\n")

            # iterate over messages
            for connection, timestamp, rawdata in reader.messages():
                if connection.topic == '/Imu0':
                    msg = deserialize_cdr(rawdata, connection.msgtype)
                    print(f"[{msg.angular_velocity.x}, {msg.angular_velocity.y}, {msg.angular_velocity.z}],\t[{msg.linear_acceleration.x}, {msg.linear_acceleration.y}, {msg.linear_acceleration.z}]")

def main(args=None):
    rclpy.init(args=args)
    
    # bag_name = input("Please enter the name of the bag file: (/home/ubuntu/FYP-ROS/rosbag/bag/)")
    # while not os.path.exists('/home/ubuntu/FYP-ROS/rosbag/bag/' + bag_name):
    #     bag_name = input("File not exist, please enter the name of the bag file: ")
        
    calculator = Calculator("test")
    rclpy.spin(calculator)

    calculator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
