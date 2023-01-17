import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

import message_filters

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu

import tf_transformations
import numpy as np
from math import cos, sin

class DTF_Calculator(Node):

    def __init__(self):
        super().__init__('dtf_calculator')

        # Get the parameters
        self.declare_parameter('finger_length', 0.15)
        self.finger_length = self.get_parameter('finger_length').get_parameter_value().double_value
        self.declare_parameter('use_current_time', False)
        self.use_current_time = self.get_parameter('use_current_time').get_parameter_value().bool_value

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create a subscriber to the topic /Imu
        self.imu_msg_filter_0 = message_filters.Subscriber(self, Imu, '/Imu0')
        self.imu_msg_filter_1 = message_filters.Subscriber(self, Imu, '/Imu1')
        self.imu_msg_filter_2 = message_filters.Subscriber(self, Imu, '/Imu2')

        self.ats1 = message_filters.TimeSynchronizer([self.imu_msg_filter_0, self.imu_msg_filter_1], 10)
        self.ats2 = message_filters.TimeSynchronizer([self.imu_msg_filter_0, self.imu_msg_filter_2], 10)
        self.ats1.registerCallback(self.cal_dtf)
        self.ats2.registerCallback(self.cal_dtf)

    def cal_dtf(self, imu_parent, imu_child):
        tf = TransformStamped()
        if self.use_current_time:
            tf.header.stamp = self.get_clock().now().to_msg()
        else:
            tf.header.stamp = imu_parent.header.stamp
        tf.header.frame_id = imu_parent.header.frame_id
        tf.child_frame_id = imu_child.header.frame_id
        
        # rpy from quaternion
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(
            np.array([imu_child.orientation.x,  imu_child.orientation.y,  imu_child.orientation.z,  imu_child.orientation.w]) -
            np.array([imu_parent.orientation.x, imu_parent.orientation.y, imu_parent.orientation.z, imu_parent.orientation.w]) 
        )

        # Calculate the rotation matrix from quaternion
        tf.transform.translation.x = self.finger_length * cos(pitch) * cos(yaw) 
        tf.transform.translation.y = self.finger_length * cos(pitch) * sin(yaw)
        tf.transform.translation.z = self.finger_length * sin(pitch)

        # Send the transform
        self.tf_broadcaster.sendTransform(tf)
        print(f"tf {tf.header.frame_id} -> {tf.child_frame_id} sent({tf.transform.translation})")

def main():
    rclpy.init()
    node = DTF_Calculator()
    rclpy.spin(node)

    node.destroy_node() 
    rclpy.shutdown()