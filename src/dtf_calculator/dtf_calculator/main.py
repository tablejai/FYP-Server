import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

import message_filters

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu

import tf_transformations
import numpy as np
from math import cos, sin, pi, sqrt, atan2, asin
 
def quaternion_rotation_matrix(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.
 
    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
    """
    # Extract the values from Q
    q0 = Q.x
    q1 = Q.y
    q2 = Q.z
    q3 = Q.w
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix

class DTF_Calculator(Node):

    def __init__(self):
        super().__init__('dtf_calculator')

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create a subscriber to the topic /Imu
        self.imu_msg_filter_0 = message_filters.Subscriber(self, Imu, '/Imu0')
        self.imu_msg_filter_1 = message_filters.Subscriber(self, Imu, '/Imu1')
        self.imu_msg_filter_2 = message_filters.Subscriber(self, Imu, '/Imu2')

        self.ats1 = message_filters.TimeSynchronizer([self.imu_msg_filter_0, self.imu_msg_filter_1], 10)
        self.ats2 = message_filters.TimeSynchronizer([self.imu_msg_filter_0, self.imu_msg_filter_2], 10)
        self.ats1.registerCallback(self.publish_dtf)
        self.ats2.registerCallback(self.publish_dtf)

        self.finger_length = 2.0

    # TODO: take origin tf into account
    def publish_dtf(self, imu_parent, imu_child):
        tf = TransformStamped()
        tf.header.stamp = imu_parent.header.stamp
        tf.header.frame_id = imu_parent.header.frame_id
        tf.child_frame_id = imu_child.header.frame_id
        
        # rpy from quaternion
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(
            np.array([imu_parent.orientation.x, imu_parent.orientation.y, imu_parent.orientation.z, imu_parent.orientation.w]) - 
            np.array([imu_child.orientation.x, imu_child.orientation.y, imu_child.orientation.z, imu_child.orientation.w])
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