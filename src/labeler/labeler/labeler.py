import rclpy
from rclpy.node import Node

# msg
import message_filters

# msg
from sensor_msgs.msg import Imu

# system
import threading
import pandas as pd
import sys

class Labeler(Node):

    def __init__(self, data_path):
        super().__init__('Labeler')
        self.imu_list = ["/Imu0", "/Imu1", "/Imu2"]
        self.data_path = data_path

        # create imu subscribers
        self.imu_msg_filter_0 = message_filters.Subscriber(self, Imu, '/Imu0')
        self.imu_msg_filter_1 = message_filters.Subscriber(self, Imu, '/Imu1')
        self.imu_msg_filter_2 = message_filters.Subscriber(self, Imu, '/Imu2')
        self.syncer = message_filters.TimeSynchronizer([self.imu_msg_filter_0, self.imu_msg_filter_1, self.imu_msg_filter_2], 10)
        self.syncer.registerCallback(self.cb)

        # create label list
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

        # create new thread to save data
        self.thread = threading.Thread(target=self.save_data)
        self.thread.start()

    def save_data(self):
        label = input(
    '''
[0]SLIDE_UP
[1]SLIDE_DOWN
[2]SLIDE_LEFT
[3]SLIDE_RIGHT
[4]ZOOM_IN
[5]ZOOM_OUT
[6]HIGHTLIGHT
[7]ON_YES
[8]OFF_NO
Enter labels: ''').strip().split(" ")
        
        self.data_df = pd.DataFrame(self.data)
        self.data_df.to_csv(f"{self.data_path}_data", index=False)
        self.label_df = pd.DataFrame({"label": label})
        self.label_df.to_csv(f"{self.data_path}_label", index=False)

        print(f"Saving data and labels to {self.data_path}.")
        print(f"Data length: {len(self.data_df)}")
        print(f"Label length: {label}")

    def cb(self, msg0, msg1, msg2):
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
        
def main(args=None):
    rclpy.init(args=args)

    if(len(sys.argv) != 2):
        print("Please enter bag name")
        print("Usage: ros2 run labeler labeler <bag_name>")
        exit(1)

    BAG_NAME = sys.argv[1]
    DATA_PATH = f"/home/ubuntu/FYP-ROS/rosbag/data/{BAG_NAME}.csv"

    labeler = Labeler(DATA_PATH)
    rclpy.spin(labeler)

    labeler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
