import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

import message_filters

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu

import tf_transformations
import numpy as np
 
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
        self.imu_msg_filter_a = message_filters.Subscriber(self, Imu, '/Imu0')
        self.imu_msg_filter_b = message_filters.Subscriber(self, Imu, '/Imu1')

        self.ats = message_filters.TimeSynchronizer([self.imu_msg_filter_a, self.imu_msg_filter_b], 10)
        self.ats.registerCallback(self.publish_dtf)

        self.vel_x = 0
        self.vel_y = 0
        self.vel_z = 0
        self.pos_x = 0
        self.pos_y = 0
        self.pos_z = 0
        self.previous_tick = 0
        self.current_tick = 0
        self.first_iter = True

    # TODO: take origin tf into account
    def publish_dtf(self, msg1, msg2):

        print("\n\n==================\nmsg received")

        # Calculate the time difference between the current and previous IMU messages
        self.current_tick = msg1.header.stamp.sec + msg1.header.stamp.nanosec * 1e-9
        dt = self.current_tick - self.previous_tick

        if self.current_tick < self.previous_tick:
            return

        if self.first_iter:
            self.previous_tick = msg1.header.stamp.sec + msg1.header.stamp.nanosec * 1e-9
            self.first_iter = False
            return

        tf = TransformStamped()
        tf.header.stamp = msg1.header.stamp
        tf.header.frame_id = msg1.header.frame_id
        tf.child_frame_id = msg2.header.frame_id
        
        # Get linear acceleration from IMU data
        acc1 = np.array([msg1.linear_acceleration.x, msg1.linear_acceleration.y, msg1.linear_acceleration.z])
        acc2 = np.array([msg2.linear_acceleration.x, msg2.linear_acceleration.y, msg2.linear_acceleration.z])

        rpy1 = tf_transformations.euler_from_quaternion([msg1.orientation.x, msg1.orientation.y, msg1.orientation.z, msg1.orientation.w])
        rpy2 = tf_transformations.euler_from_quaternion([msg2.orientation.x, msg2.orientation.y, msg2.orientation.z, msg2.orientation.w])
        print(f"{msg1.header.frame_id} acc: {acc1} orientation: {rpy1}")
        print(f"{msg2.header.frame_id} acc: {acc2} orientation: {rpy2}")
        
        # Convert the linear acceleration to the global reference frame
        acc1 = np.dot(quaternion_rotation_matrix(msg1.orientation), acc1)
        acc2 = np.dot(quaternion_rotation_matrix(msg2.orientation), acc2)
        
        print("rotation matrix applied")
        print(f"{msg1.header.frame_id} acc:  ", acc1)
        print(f"{msg2.header.frame_id} acc:  ", acc2)

        # Calculate the difference between the two accelerations
        acc_x, acc_y, acc_z = acc1 - acc2

        # Integrate acceleration to estimate velocity
        self.vel_x += acc_x * dt
        self.vel_y += acc_y * dt
        self.vel_z += acc_z * dt

        # Integrate velocity to estimate position
        self.pos_x += self.vel_x * dt
        self.pos_y += self.vel_y * dt
        self.pos_z += self.vel_z * dt

        # Set the transform's translation
        tf.transform.translation.x = self.pos_x
        tf.transform.translation.y = self.pos_y
        tf.transform.translation.z = self.pos_z

        # Send the transform
        self.tf_broadcaster.sendTransform(tf)
        print("tf:", tf.transform.translation)
        self.previous_tick = self.current_tick

def main():
    rclpy.init()
    node = DTF_Calculator()
    rclpy.spin(node)

    node.destroy_node() 
    rclpy.shutdown()