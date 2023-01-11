import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

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
        self.subscription = self.create_subscription(Imu, '/Imu0', self.publish_dtf, 1)
        self.vel_x = 0
        self.vel_y = 0
        self.vel_z = 0
        self.pos_x = 0
        self.pos_y = 0
        self.pos_z = 0
        self.previous_tick = 0
        self.current_tick = 0
        self.first_iter = True

    def publish_dtf(self, msg):
        print("\n\n==================\nmsg received")
        if(self.first_iter):
            self.previous_tick = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.first_iter = False
            return

        t = TransformStamped()

        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'ground'
        t.child_frame_id = 'imu0'
        
        # Get linear acceleration from IMU data
        acc_x = msg.linear_acceleration.x
        acc_y = msg.linear_acceleration.y
        acc_z = msg.linear_acceleration.z

        # Get orientation from IMU data
        rot_matrix = quaternion_rotation_matrix(msg.orientation)
        acc_x, acc_y, acc_z = np.dot(rot_matrix, np.array([acc_x, acc_y, acc_z]))
        print(f"acc:{acc_x}, {acc_y}, {acc_z}")
    
        # Calculate the time difference between the current and previous IMU messages
        self.current_tick = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = self.current_tick - self.previous_tick

        # Integrate acceleration to estimate velocity
        self.vel_x += acc_x * dt
        self.vel_y += acc_y * dt
        self.vel_z += acc_z * dt

        # Integrate velocity to estimate position
        self.pos_x += self.vel_x * dt
        self.pos_y += self.vel_y * dt
        self.pos_z += self.vel_z * dt

        # Set the transform's translation
        t.transform.translation.x = self.pos_x
        t.transform.translation.y = self.pos_y
        t.transform.translation.z = self.pos_z

        # Send the transform
        self.tf_broadcaster.sendTransform(t)
        print("tf:", t.transform.translation)
        self.previous_tick = self.current_tick

def main():
    rclpy.init()
    node = DTF_Calculator()
    rclpy.spin(node)

    rclpy.shutdown()