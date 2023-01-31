import rclpy
from rclpy.node import Node

# msg filter
from message_filters import Subscriber, SimpleFilter, TimeSynchronizer

# msg
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage

# system
import threading
import pandas as pd
import sys

class TfSubscriber(SimpleFilter):
    def __init__(self, node, message_type, topic, from_frame, to_frame):
        SimpleFilter.__init__(self)
        self.node = node
        self.message_type = message_type
        self.topic = topic 
        self.from_frame = from_frame
        self.to_frame = to_frame
        self.sub = self.node.create_subscription(self.message_type, self.topic, self.callback, 10)

    def callback(self, msg):
        for tf in msg.transforms:
            if tf.header.frame_id == self.from_frame and tf.child_frame_id == self.to_frame:
                self.signalMessage(tf)

class Labeler(Node):

    def __init__(self):
        super().__init__('Labeler')
        self.declare_parameter('bag_path')
        self.data_path = f"/home/ubuntu/FYP-ROS/rosbag/data/{self.get_parameter('bag_path').value}"
        print("data path:", self.data_path)

        self.imu_list = ["Imu0", "Imu1", "Imu2"]
        self.tf_list = ["imu0_to_imu1", "imu0_to_imu2"]

        # create imu subscribers
        self.imu_msg_filter0 = Subscriber(self, Imu, '/Imu0')
        self.imu_msg_filter1 = Subscriber(self, Imu, '/Imu1')
        self.imu_msg_filter2 = Subscriber(self, Imu, '/Imu2')
        self.tf_msg_filter1  = TfSubscriber(self, TFMessage, '/tf', from_frame='imu0', to_frame='imu1')
        self.tf_msg_filter2  = TfSubscriber(self, TFMessage, '/tf', from_frame='imu0', to_frame='imu2')
        
        self.syncer = TimeSynchronizer([self.imu_msg_filter0, self.imu_msg_filter1, self.imu_msg_filter2, self.tf_msg_filter1, self.tf_msg_filter2], 10)
        self.syncer.registerCallback(self.sync_callback)

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

        # create new thread to save data
        self.thread = threading.Thread(target=self.save_data)
        self.thread.start()

    def save_data(self):
        label = input(
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
Enter labels: ''').strip().split(" ")
        
        self.data_df = pd.DataFrame(self.data)
        self.data_df.to_csv(f"{self.data_path}_data.csv", index=False)
        self.label_df = pd.DataFrame({"label": label})
        self.label_df.to_csv(f"{self.data_path}_label.csv", index=False)

        print(f"Saving data and labels to {self.data_path}...")
        print(f"Data length: {len(self.data_df)}")
        print(f"Label: {label}")

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
