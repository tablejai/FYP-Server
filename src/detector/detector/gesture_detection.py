import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage
from msgs.msg import Geasture

from message_filters import Subscriber, TimeSynchronizer
from utils.tf_subscriber import TfSubscriber

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from tensorflow import keras

class Detector(Node):
    def __init__(self):
        super().__init__('Detector')

        # create subscribers
        self.imu_list = ["Imu0", "Imu1", "Imu2"]
        self.tf_list = ["imu0_to_imu1", "imu0_to_imu2"]

        self.imu_subscribers = []
        self.tf_subscribers = []
        for imu in self.imu_list:
            self.imu_subscribers.append(Subscriber(self, Imu, imu))
        for tf in self.tf_list:
            self.tf_subscribers.append(TfSubscriber(self, TFMessage, '/tf', from_frame='imu0', to_frame='imu1'))

        self.syncer = TimeSynchronizer([fs for fs in self.imu_subscribers + self.tf_subscribers], 10)
        self.syncer.registerCallback(self.sync_callback)

        # create a publisher
        self.command_publisher = self.create_publisher(Geasture, '/Geastures', 10)

        # create a timer to do inferencing
        self.timer = self.create_timer(2, self.inference)

        # create buffer
        self.data_queue = pd.DataFrame(
            columns=[
                f"{self.imu_list[0]}_linear_accleration_x", f"{self.imu_list[0]}_linear_accleration_y", f"{self.imu_list[0]}_linear_accleration_z",
                f"{self.imu_list[0]}_angular_velocity_x", f"{self.imu_list[0]}_angular_velocity_y", f"{self.imu_list[0]}_angular_velocity_z",
                f"{self.imu_list[0]}_orientation_x", f"{self.imu_list[0]}_orientation_y", f"{self.imu_list[0]}_orientation_z", f"{self.imu_list[0]}_orientation_w",
                f"{self.imu_list[1]}_linear_accleration_x", f"{self.imu_list[1]}_linear_accleration_y", f"{self.imu_list[1]}_linear_accleration_z",
                f"{self.imu_list[1]}_angular_velocity_x", f"{self.imu_list[1]}_angular_velocity_y", f"{self.imu_list[1]}_angular_velocity_z",
                f"{self.imu_list[1]}_orientation_x", f"{self.imu_list[1]}_orientation_y", f"{self.imu_list[1]}_orientation_z", f"{self.imu_list[1]}_orientation_w",
                f"{self.imu_list[2]}_linear_accleration_x", f"{self.imu_list[2]}_linear_accleration_y", f"{self.imu_list[2]}_linear_accleration_z",
                f"{self.imu_list[2]}_angular_velocity_x", f"{self.imu_list[2]}_angular_velocity_y", f"{self.imu_list[2]}_angular_velocity_z", 
                f"{self.imu_list[2]}_orientation_x", f"{self.imu_list[2]}_orientation_y", f"{self.imu_list[2]}_orientation_z", f"{self.imu_list[2]}_orientation_w",
                f"{self.tf_list[0]}_translation_x", f"{self.tf_list[0]}_translation_y", f"{self.tf_list[0]}_translation_z", 
                f"{self.tf_list[1]}_translation_x", f"{self.tf_list[1]}_translation_y", f"{self.tf_list[1]}_translation_z",
            ]
        )
    
        # load the pretrain lstm model
        self.model = keras.models.load_model("/home/ubuntu/FYP-ROS/weights/model_lstm-2023_2_26-3_54-acc0.98")
        self.DATA_BUF_LEN = 250
        self.get_logger().info("model loaded")

    def sync_callback(self, *msgs):
        d = {}
        for i, msg in enumerate(msgs):
            if i < len(self.imu_list):
                imu_num = i
                d[f"{self.imu_list[imu_num]}_linear_accleration_x"] = [msg.linear_acceleration.x]
                d[f"{self.imu_list[imu_num]}_linear_accleration_y"] = [msg.linear_acceleration.y]
                d[f"{self.imu_list[imu_num]}_linear_accleration_z"] = [msg.linear_acceleration.z]
                d[f"{self.imu_list[imu_num]}_angular_velocity_x"] = [msg.angular_velocity.x]
                d[f"{self.imu_list[imu_num]}_angular_velocity_y"] = [msg.angular_velocity.y]
                d[f"{self.imu_list[imu_num]}_angular_velocity_z"] = [msg.angular_velocity.z]
                d[f"{self.imu_list[imu_num]}_orientation_x"] = [msg.orientation.x]
                d[f"{self.imu_list[imu_num]}_orientation_y"] = [msg.orientation.y]
                d[f"{self.imu_list[imu_num]}_orientation_z"] = [msg.orientation.z]
                d[f"{self.imu_list[imu_num]}_orientation_w"] = [msg.orientation.w]
            elif i < len(self.imu_list) + len(self.tf_list):
                tf_num = i - len(self.imu_list)
                d[f"{self.tf_list[tf_num]}_translation_x"] = [msg.transform.translation.x]
                d[f"{self.tf_list[tf_num]}_translation_y"] = [msg.transform.translation.y]
                d[f"{self.tf_list[tf_num]}_translation_z"] = [msg.transform.translation.z]

        self.data_queue = pd.concat([self.data_queue, pd.DataFrame(data=d)], ignore_index=True)

    def inference(self):
        df_row_cnt = self.data_queue.shape[0]        
        if df_row_cnt == 0:
            return

        # prepare data
        X_data = self.data_queue.to_numpy()
        if df_row_cnt <  self.DATA_BUF_LEN:
            X_data = np.pad(X_data, ((0,  self.DATA_BUF_LEN - X_data.shape[0]), (0, 0)), 'constant')
        else:
            X_data = X_data[-1 * self.DATA_BUF_LEN:]

        # plot data
        # fig, axs = plt.subplots(3, 6)
        # axs[0, 0].plot(X_data[:, 1])
        # axs[0, 1].plot(X_data[:, 2])
        # axs[0, 2].plot(X_data[:, 3])

        # do prediction
        y_pred = self.model.predict(np.expand_dims(X_data, axis=0))
        y_label = np.argmax(y_pred, axis=1)[0]
        print(f"{y_pred=}")
        print(f"prediction: {y_label} (possibility: {np.max(y_pred, axis=1)[0]})")

        # publish command
        gestures = {
            Geasture.STATIC: Geasture(type=Geasture.STATIC),
            Geasture.SLIDE_UP: Geasture(type=Geasture.SLIDE_UP),
            Geasture.SLIDE_DOWN: Geasture(type=Geasture.SLIDE_DOWN),
            Geasture.SLIDE_LEFT: Geasture(type=Geasture.SLIDE_LEFT),
            Geasture.SLIDE_RIGHT: Geasture(type=Geasture.SLIDE_RIGHT),
            Geasture.ZOOM_IN: Geasture(type=Geasture.ZOOM_IN),
            Geasture.ZOOM_OUT: Geasture(type=Geasture.ZOOM_OUT),
            Geasture.NONE: Geasture(type=Geasture.NONE)
        }
        gesture = gestures.get(y_label)
        if gesture:
            self.command_publisher.publish(gesture)

def main():
    rclpy.init()
    node = Detector()
    rclpy.spin(node)

    node.destroy_node() 
    rclpy.shutdown()

if __name__ == '__main__':
    main()
