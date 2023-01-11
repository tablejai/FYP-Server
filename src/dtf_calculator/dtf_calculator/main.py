import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from turtlesim.msg import Pose

import numpy as np
import math
import tf_transformations

class FramePublisher(Node):

    def __init__(self):
        super().__init__('turtle_tf2_frame_publisher')

        # Not sure if I really need one or not
        # Declare and acquire `imuAugmentedTopic` parameter
        self.imuName = self.declare_parameter(
            'imuName', 'imu_palm').get_parameter_value().string_value

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
        # callback function on each message
        self.subscription = self.create_subscription(
            Pose,
            '/imuAugmentedArray',
            self.handle_turtle_pose,
            1)
        self.subscription  # prevent unused variable warning

    def handle_turtle_pose(self, msg):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = self.imuName

        t.transform.translation.x = msg.linear_trans_x
        t.transform.translation.y = msg.linear_trans_y
        t.transform.translation.z = msg.linear_trans_z

        t.transform.rotation.x = msg.quaternion_x
        t.transform.rotation.y = msg.quaternion_y
        t.transform.rotation.z = msg.quaternion_z
        t.transform.rotation.w = msg.quaternion_w

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()