import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage
from msgs.msg import Gesture

from message_filters import Subscriber, TimeSynchronizer
from utils.tf_subscriber import TfSubscriber

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from tensorflow import keras

model_path = "/home/ubuntu/FYP-Glove/weights/model_lstm-2023_4_7-9_5-acc1.00.h5"
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
        self.command_publisher = self.create_publisher(Gesture, '/Gestures', 10)

        # create a timer to do inferencing
        infer_interval = 0.4
        self.infer_timer = self.create_timer(infer_interval, self.inference)
        self.buf_timer = self.create_timer(10, self.clean_buffer)
        self.plt_timer = self.create_timer(infer_interval, self.plot_buffer)
        # self.fig, self.ax = plt.subplots(6, 3)

        # create buffer
        self.data_queue = pd.DataFrame(
            columns=[
                f"{self.imu_list[0]}_linear_acceleration_x", f"{self.imu_list[0]}_linear_acceleration_y", f"{self.imu_list[0]}_linear_acceleration_z",
                # f"{self.imu_list[0]}_angular_velocity_x", f"{self.imu_list[0]}_angular_velocity_y", f"{self.imu_list[0]}_angular_velocity_z",
                # f"{self.imu_list[0]}_orientation_x", f"{self.imu_list[0]}_orientation_y", f"{self.imu_list[0]}_orientation_z", f"{self.imu_list[0]}_orientation_w",
                f"{self.imu_list[1]}_linear_acceleration_x", f"{self.imu_list[1]}_linear_acceleration_y", f"{self.imu_list[1]}_linear_acceleration_z",
                # f"{self.imu_list[1]}_angular_velocity_x", f"{self.imu_list[1]}_angular_velocity_y", f"{self.imu_list[1]}_angular_velocity_z",
                # f"{self.imu_list[1]}_orientation_x", f"{self.imu_list[1]}_orientation_y", f"{self.imu_list[1]}_orientation_z", f"{self.imu_list[1]}_orientation_w",
                f"{self.imu_list[2]}_linear_acceleration_x", f"{self.imu_list[2]}_linear_acceleration_y", f"{self.imu_list[2]}_linear_acceleration_z",
                # f"{self.imu_list[2]}_angular_velocity_x", f"{self.imu_list[2]}_angular_velocity_y", f"{self.imu_list[2]}_angular_velocity_z", 
                # f"{self.imu_list[2]}_orientation_x", f"{self.imu_list[2]}_orientation_y", f"{self.imu_list[2]}_orientation_z", f"{self.imu_list[2]}_orientation_w",
                # f"{self.tf_list[0]}_translation_x", f"{self.tf_list[0]}_translation_y", f"{self.tf_list[0]}_translation_z", 
                # f"{self.tf_list[1]}_translation_x", f"{self.tf_list[1]}_translation_y", f"{self.tf_list[1]}_translation_z",
            ]
        )
    
        # load the pretrain lstm model
        self.model = keras.models.load_model(model_path)
        self.DATA_BUF_LEN = 50
        self.get_logger().info("model loaded")

    def sync_callback(self, *msgs):
        d = {}
        for i, msg in enumerate(msgs):
            if i < len(self.imu_list):
                imu_num = i
                d[f"{self.imu_list[imu_num]}_linear_acceleration_x"] = [msg.linear_acceleration.x]
                d[f"{self.imu_list[imu_num]}_linear_acceleration_y"] = [msg.linear_acceleration.y]
                d[f"{self.imu_list[imu_num]}_linear_acceleration_z"] = [msg.linear_acceleration.z]
                # d[f"{self.imu_list[imu_num]}_angular_velocity_x"] = [msg.angular_velocity.x]
                # d[f"{self.imu_list[imu_num]}_angular_velocity_y"] = [msg.angular_velocity.y]
                # d[f"{self.imu_list[imu_num]}_angular_velocity_z"] = [msg.angular_velocity.z]
                # d[f"{self.imu_list[imu_num]}_orientation_x"] = [msg.orientation.x]
                # d[f"{self.imu_list[imu_num]}_orientation_y"] = [msg.orientation.y]
                # d[f"{self.imu_list[imu_num]}_orientation_z"] = [msg.orientation.z]
                # d[f"{self.imu_list[imu_num]}_orientation_w"] = [msg.orientation.w]
            elif i < len(self.imu_list) + len(self.tf_list):
                tf_num = i - len(self.imu_list)
                # d[f"{self.tf_list[tf_num]}_translation_x"] = [msg.transform.translation.x]
                # d[f"{self.tf_list[tf_num]}_translation_y"] = [msg.transform.translation.y]
                # d[f"{self.tf_list[tf_num]}_translation_z"] = [msg.transform.translation.z]

        self.data_queue = pd.concat([self.data_queue, pd.DataFrame(data=d)], ignore_index=True)

    def inference(self):
        data_len = self.data_queue.shape[0]        
        if data_len == 0:
            return

        # prepare data
        X = self.data_queue.to_numpy()
        if data_len < self.DATA_BUF_LEN:
            last_row = np.repeat([X[-1]], repeats=self.DATA_BUF_LEN-data_len, axis=0)
            X = np.vstack([X, last_row])
        else:
            X = X[-1 * self.DATA_BUF_LEN:]
        
        # do prediction
        y_pred = self.model.predict(np.expand_dims(X, axis=0))
        y_label = np.argmax(y_pred, axis=1)[0]
        probability = np.max(y_pred, axis=1)[0]
        # self.get_logger().info(f"{y_pred=}")
        self.get_logger().info(f"prediction: {y_label} (probability: {np.max(y_pred, axis=1)[0]})")

        # publish command
        gestures = {
            Gesture.STATIC: Gesture(type=Gesture.STATIC),
            Gesture.SLIDE_UP: Gesture(type=Gesture.SLIDE_UP),
            Gesture.SLIDE_DOWN: Gesture(type=Gesture.SLIDE_DOWN),
            Gesture.SLIDE_LEFT: Gesture(type=Gesture.SLIDE_LEFT),
            Gesture.SLIDE_RIGHT: Gesture(type=Gesture.SLIDE_RIGHT),
            Gesture.RELEASE: Gesture(type=Gesture.RELEASE),
            Gesture.GRASP: Gesture(type=Gesture.GRASP),
            Gesture.NONE: Gesture(type=Gesture.NONE),
            Gesture.POINTING: Gesture(type=Gesture.POINTING),
        }
        gesture = gestures.get(y_label)
        if gesture and probability > 0.9:
            self.command_publisher.publish(gesture)

    def clean_buffer(self):
        data_len = self.data_queue.shape[0]
        if data_len > self.DATA_BUF_LEN * 1.5:
            self.data_queue = self.data_queue.iloc[-1 * self.DATA_BUF_LEN:]

    def plot_buffer(self):
        X = self.data_queue.iloc[-1 * self.DATA_BUF_LEN:]

        fig = plt.figure(1)
        fig.clf()

        plt.subplot(3, 1, 1)
        plt.plot(X["Imu0_linear_acceleration_x"], 'r')
        plt.ylim(-1.5 * 9.8, 1.5 * 9.8)
        plt.ylabel("Imu0")

        plt.subplot(3, 1, 2)
        plt.plot(X["Imu1_linear_acceleration_x"], 'r')
        plt.ylim(-1.5 * 9.8, 1.5 * 9.8)
        plt.ylabel("Imu1")

        plt.subplot(3, 1, 3)
        plt.plot(X["Imu2_linear_acceleration_x"], 'r')
        plt.ylim(-1.5 * 9.8, 1.5 * 9.8)
        plt.ylabel("Imu2")

        plt.plot([], [], ' ', label="x:sample")
        plt.plot([], [], ' ', label="y:m/s^2")
        fig.legend()
        
        plt.draw()
        plt.pause(0.00000000001)
        
def main():
    rclpy.init()
    node = Detector()

    plt.ion()
    plt.show()
    rclpy.spin(node)

    node.destroy_node() 
    rclpy.shutdown()

if __name__ == '__main__':
    main()
