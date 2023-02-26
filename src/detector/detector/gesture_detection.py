import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage
from msgs.msg import Geasture

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from message_filters import Subscriber, SimpleFilter, TimeSynchronizer, ApproximateTimeSynchronizer

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from tensorflow import keras
# from tensorflow.keras import layers
# from tensorflow.keras.preprocessing import sequence
# from keras.models import Sequential
# from keras.layers import Activation, Dense, Dropout, LSTM, Softmax, Bidirectional
# from keras.callbacks import EarlyStopping
# from sklearn.model_selection import train_test_split
# from sklearn.metrics import confusion_matrix

class TfSubscriber(SimpleFilter):
    def __init__(self, node, message_type, topic, from_frame, to_frame):
        SimpleFilter.__init__(self)
        self.node = node
        self.message_type = message_type
        self.topic = topic 
        self.from_frame = from_frame
        self.to_frame = to_frame
        self.sub = self.node.create_subscription(self.message_type, self.topic, self.callback, 100)

    def callback(self, msg):
        for tf in msg.transforms:
            if tf.header.frame_id == self.from_frame and tf.child_frame_id == self.to_frame:
                self.signalMessage(tf)

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
                f"{self.imu_list[0]}_linear_accleration_x",
                f"{self.imu_list[0]}_linear_accleration_y",
                f"{self.imu_list[0]}_linear_accleration_z",
                f"{self.imu_list[0]}_angular_velocity_x",
                f"{self.imu_list[0]}_angular_velocity_y",
                f"{self.imu_list[0]}_angular_velocity_z",
                f"{self.imu_list[0]}_orientation_x",
                f"{self.imu_list[0]}_orientation_y",
                f"{self.imu_list[0]}_orientation_z",
                f"{self.imu_list[0]}_orientation_w",
                f"{self.imu_list[1]}_linear_accleration_x",
                f"{self.imu_list[1]}_linear_accleration_y",
                f"{self.imu_list[1]}_linear_accleration_z",
                f"{self.imu_list[1]}_angular_velocity_x",
                f"{self.imu_list[1]}_angular_velocity_y",
                f"{self.imu_list[1]}_angular_velocity_z",
                f"{self.imu_list[1]}_orientation_x",
                f"{self.imu_list[1]}_orientation_y",
                f"{self.imu_list[1]}_orientation_z",
                f"{self.imu_list[1]}_orientation_w",
                f"{self.imu_list[2]}_linear_accleration_x",
                f"{self.imu_list[2]}_linear_accleration_y",
                f"{self.imu_list[2]}_linear_accleration_z",
                f"{self.imu_list[2]}_angular_velocity_x",
                f"{self.imu_list[2]}_angular_velocity_y",
                f"{self.imu_list[2]}_angular_velocity_z",
                f"{self.imu_list[2]}_orientation_x",
                f"{self.imu_list[2]}_orientation_y",
                f"{self.imu_list[2]}_orientation_z",
                f"{self.imu_list[2]}_orientation_w",
                f"{self.tf_list[0]}_translation_x",
                f"{self.tf_list[0]}_translation_y",
                f"{self.tf_list[0]}_translation_z",
                f"{self.tf_list[1]}_translation_x",
                f"{self.tf_list[1]}_translation_y",
                f"{self.tf_list[1]}_translation_z",
            ]
        )
    
        # create lstm model
        self.model = keras.models.load_model("/home/ubuntu/FYP-ROS/weights/model_lstm-2023_2_26-3_47-acc0.96")
        self.DATA_BUF_LEN = 500

    def sync_callback(self, imu_msg0, imu_msg1, imu_msg2, tf_msg0, tf_msg1):
        d = {   
            # "timestamp": [imu_msg0.header.stamp.sec + imu_msg0.header.stamp.nanosec*1e-9],
            f"{self.imu_list[0]}_linear_accleration_x": [imu_msg0.linear_acceleration.x],
            f"{self.imu_list[0]}_linear_accleration_y": [imu_msg0.linear_acceleration.y],
            f"{self.imu_list[0]}_linear_accleration_z": [imu_msg0.linear_acceleration.z],
            f"{self.imu_list[0]}_angular_velocity_x": [imu_msg0.angular_velocity.x],
            f"{self.imu_list[0]}_angular_velocity_y": [imu_msg0.angular_velocity.y],
            f"{self.imu_list[0]}_angular_velocity_z": [imu_msg0.angular_velocity.z],
            f"{self.imu_list[0]}_orientation_x": [imu_msg0.orientation.x],
            f"{self.imu_list[0]}_orientation_y": [imu_msg0.orientation.y],
            f"{self.imu_list[0]}_orientation_z": [imu_msg0.orientation.z],
            f"{self.imu_list[0]}_orientation_w": [imu_msg0.orientation.w],
            f"{self.imu_list[1]}_linear_accleration_x": [imu_msg1.linear_acceleration.x],
            f"{self.imu_list[1]}_linear_accleration_y": [imu_msg1.linear_acceleration.y],
            f"{self.imu_list[1]}_linear_accleration_z": [imu_msg1.linear_acceleration.z],
            f"{self.imu_list[1]}_angular_velocity_x": [imu_msg1.angular_velocity.x],
            f"{self.imu_list[1]}_angular_velocity_y": [imu_msg1.angular_velocity.y],
            f"{self.imu_list[1]}_angular_velocity_z": [imu_msg1.angular_velocity.z],
            f"{self.imu_list[1]}_orientation_x": [imu_msg1.orientation.x],
            f"{self.imu_list[1]}_orientation_y": [imu_msg1.orientation.y],
            f"{self.imu_list[1]}_orientation_z": [imu_msg1.orientation.z],
            f"{self.imu_list[1]}_orientation_w": [imu_msg1.orientation.w],
            f"{self.imu_list[2]}_linear_accleration_x": [imu_msg2.linear_acceleration.x],
            f"{self.imu_list[2]}_linear_accleration_y": [imu_msg2.linear_acceleration.y],
            f"{self.imu_list[2]}_linear_accleration_z": [imu_msg2.linear_acceleration.z],
            f"{self.imu_list[2]}_angular_velocity_x": [imu_msg2.angular_velocity.x],
            f"{self.imu_list[2]}_angular_velocity_y": [imu_msg2.angular_velocity.y],
            f"{self.imu_list[2]}_angular_velocity_z": [imu_msg2.angular_velocity.z],
            f"{self.imu_list[2]}_orientation_x": [imu_msg2.orientation.x],
            f"{self.imu_list[2]}_orientation_y": [imu_msg2.orientation.y],
            f"{self.imu_list[2]}_orientation_z": [imu_msg2.orientation.z],
            f"{self.imu_list[2]}_orientation_w": [imu_msg2.orientation.w],
            f"{self.tf_list[0]}_translation_x": [tf_msg0.transform.translation.x],
            f"{self.tf_list[0]}_translation_y": [tf_msg0.transform.translation.y],
            f"{self.tf_list[0]}_translation_z": [tf_msg0.transform.translation.z],
            f"{self.tf_list[1]}_translation_x": [tf_msg1.transform.translation.x],
            f"{self.tf_list[1]}_translation_y": [tf_msg1.transform.translation.y],
            f"{self.tf_list[1]}_translation_z": [tf_msg1.transform.translation.z],
        }
        self.data_queue = pd.concat([self.data_queue, pd.DataFrame(data=d)], ignore_index=True)

    
    def inference(self):
        if len(self.data_queue["Imu0_linear_accleration_x"]) == 0:
            return

        # prepare data
        X_data = self.data_queue.to_numpy()
        if len(self.data_queue["Imu0_linear_accleration_x"]) <  self.DATA_BUF_LEN:
            X_data = np.pad(X_data, ((0,  self.DATA_BUF_LEN - X_data.shape[0]), (0, 0)), 'constant')
        else:
            X_data = X_data[-1*self.DATA_BUF_LEN:]
        X_data = X_data.astype(np.float32)

        # plot data
        # fig, axs = plt.subplots(3, 6)
        # axs[0, 0].plot(X_data[:, 1])
        # axs[0, 1].plot(X_data[:, 2])
        # axs[0, 2].plot(X_data[:, 3])

        # do prediction
        y_pred = self.model.predict(np.expand_dims(X_data, axis=0))
        y_lable = np.argmax(y_pred, axis=1)
        print(f"{y_pred=}")
        print(f"prediction: {y_lable[0]} (possibility: {np.max(y_pred, axis=1)[0]})")

        # publish command
        if y_lable == Geasture.STATIC:
            self.command_publisher.publish(Geasture(type=Geasture.STATIC))
        elif y_lable == Geasture.SLIDE_UP:
            self.command_publisher.publish(Geasture(type=Geasture.SLIDE_UP))
        elif y_lable == Geasture.SLIDE_DOWN:
            self.command_publisher.publish(Geasture(type=Geasture.SLIDE_DOWN))
        elif y_lable == Geasture.SLIDE_LEFT:
            self.command_publisher.publish(Geasture(type=Geasture.SLIDE_LEFT))
        elif y_lable == Geasture.SLIDE_RIGHT:
            self.command_publisher.publish(Geasture(type=Geasture.SLIDE_RIGHT))
        elif y_lable == Geasture.ZOOM_IN:
            self.command_publisher.publish(Geasture(type=Geasture.ZOOM_IN))
        elif y_lable == Geasture.ZOOM_OUT:
            self.command_publisher.publish(Geasture(type=Geasture.ZOOM_OUT))
        elif y_lable == Geasture.NONE:
            self.command_publisher.publish(Geasture(type=Geasture.NONE))
        else:
            pass

def main():
    rclpy.init()
    node = Detector()
    rclpy.spin(node)

    node.destroy_node() 
    rclpy.shutdown()

if __name__ == '__main__':
    main()
