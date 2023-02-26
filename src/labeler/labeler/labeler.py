import rclpy
from rclpy.node import Node

# msg filter
from message_filters import Subscriber, TimeSynchronizer

# msg
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage

from utils.tf_subscriber import TfSubscriber

# system
import threading
import pandas as pd
import sys

class Labeler(Node):

    def __init__(self):
        super().__init__('Labeler')
        self.declare_parameter('bag_name')
        self.data_path = f"/home/ubuntu/FYP-ROS/rosbag/data/data/{self.get_parameter('bag_name').value}"
        self.label_path = f"/home/ubuntu/FYP-ROS/rosbag/data/label/{self.get_parameter('bag_name').value}"
        self.get_logger().info(f"data path: {self.data_path}")
        self.get_logger().info(f"label path: {self.label_path}")

        self.imu_list = ["Imu0", "Imu1", "Imu2"]
        self.tf_list = ["imu0_to_imu1", "imu0_to_imu2"]

        # create imu subscribers
        self.imu_msg_filter0 = Subscriber(self, Imu, '/Imu0')
        self.imu_msg_filter1 = Subscriber(self, Imu, '/Imu1')
        self.imu_msg_filter2 = Subscriber(self, Imu, '/Imu2')
        self.tf_msg_filter1  = TfSubscriber(self, TFMessage, '/tf', from_frame='imu0', to_frame='imu1')
        self.tf_msg_filter2  = TfSubscriber(self, TFMessage, '/tf', from_frame='imu0', to_frame='imu2')

        self.imu_msg_filter0.registerCallback(self.debug_cb, "Imu0")
        self.imu_msg_filter1.registerCallback(self.debug_cb, "Imu1")
        self.imu_msg_filter2.registerCallback(self.debug_cb, "Imu2")
        self.tf_msg_filter1.registerCallback(self.debug_cb, "tf1")
        self.tf_msg_filter2.registerCallback(self.debug_cb, "tf2")
        
        self.syncer = TimeSynchronizer([self.imu_msg_filter0, self.imu_msg_filter1, self.imu_msg_filter2, self.tf_msg_filter1, self.tf_msg_filter2], 100)
        self.syncer.registerCallback(self.sync_callback)

        # create label listener
        self.label_sub = self.create_subscription(String, '/User_Label', self.label_callback, 10)

        # create data dict
        self.data = {}
        self.data["timestamp"] = []
        for imu in self.imu_list:
            self.data[f"{imu}_linear_accleration_x"] = []
            self.data[f"{imu}_linear_accleration_y"] = []
            self.data[f"{imu}_linear_accleration_z"] = []
            self.data[f"{imu}_angular_velocity_x"] = []
            self.data[f"{imu}_angular_velocity_y"] = []
            self.data[f"{imu}_angular_velocity_z"] = []
            self.data[f"{imu}_orientation_x"] = []
            self.data[f"{imu}_orientation_y"] = []
            self.data[f"{imu}_orientation_z"] = []
            self.data[f"{imu}_orientation_w"] = []
        for tf in self.tf_list:
            self.data[f"{tf}_translation_x"] = []
            self.data[f"{tf}_translation_y"] = []
            self.data[f"{tf}_translation_z"] = []
            self.data[f"{tf}_rotation_x"] = []
            self.data[f"{tf}_rotation_y"] = []
            self.data[f"{tf}_rotation_z"] = []
            self.data[f"{tf}_rotation_w"] = []

        # log
        self.get_logger().info(
'''
[0]STATIC
[1]SLIDE_UP
[2]SLIDE_DOWN
[3]SLIDE_LEFT
[4]SLIDE_RIGHT
[5]ZOOM_IN
[6]ZOOM_OUT
[7]HIGHLIGHT
[8]ON_YES
[9]OFF_NO
'''
        )
    def debug_cb(self, msg, name):
        self.get_logger().info(f"{name}:\t{msg.header.stamp.sec}.{msg.header.stamp.nanosec}")

    def label_callback(self, labels):
        labels = labels.data.strip().split(" ")
        
        self.data_df = pd.DataFrame(self.data)
        self.data_df.to_csv(f"{self.data_path}_data.csv", index=False)
        self.label_df = pd.DataFrame({"label": labels})
        self.label_df.to_csv(f"{self.label_path}_label.csv", index=False)

        self.get_logger().info(f"Saving data to {self.data_path} and label to {self.label_path}...")
        self.get_logger().info(f"Data length: {len(self.data_df)}")
        self.get_logger().info(f"Label: {labels}")

    def sync_callback(self, msg0, msg1, msg2, msg3, msg4):
        self.data[f"timestamp"].append(msg0.header.stamp.sec + msg0.header.stamp.nanosec*1e-9)
        for imu, msg in zip(self.imu_list, [msg0, msg1, msg2]):
            self.data[f"{imu}_linear_accleration_x"].append(msg.linear_acceleration.x)
            self.data[f"{imu}_linear_accleration_y"].append(msg.linear_acceleration.y)
            self.data[f"{imu}_linear_accleration_z"].append(msg.linear_acceleration.z)
            self.data[f"{imu}_angular_velocity_x"].append(msg.angular_velocity.x)
            self.data[f"{imu}_angular_velocity_y"].append(msg.angular_velocity.y)
            self.data[f"{imu}_angular_velocity_z"].append(msg.angular_velocity.z)
            self.data[f"{imu}_orientation_x"].append(msg.orientation.x)
            self.data[f"{imu}_orientation_y"].append(msg.orientation.y)
            self.data[f"{imu}_orientation_z"].append(msg.orientation.z)
            self.data[f"{imu}_orientation_w"].append(msg.orientation.w)

        for tf, msg in zip(self.tf_list, [msg3, msg4]):
            self.data[f"{tf}_translation_x"].append(msg.transform.translation.x)
            self.data[f"{tf}_translation_y"].append(msg.transform.translation.y)
            self.data[f"{tf}_translation_z"].append(msg.transform.translation.z)
            self.data[f"{tf}_rotation_x"].append(msg.transform.rotation.x)
            self.data[f"{tf}_rotation_y"].append(msg.transform.rotation.y)
            self.data[f"{tf}_rotation_z"].append(msg.transform.rotation.z)
            self.data[f"{tf}_rotation_w"].append(msg.transform.rotation.w)

def main(args=None):
    rclpy.init(args=args)

    labeler = Labeler()
    rclpy.spin(labeler)

    labeler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
