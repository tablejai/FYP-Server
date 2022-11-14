import rclpy
from rclpy.node import Node
# from msgs.msg import ImuRawArray, ImuAugmentedHeadless, ImuAugmentedArray    
import random

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion
import tf_transformations
import math

raw_topic = '/Imu_raw0'
augmented_topic = '/Imu_aug0'

class AugmentNode(Node):

    def __init__(self):
        super().__init__('imu_subscriber')
        
        self.subscriber_ = self.create_subscription(Imu, raw_topic, self.listener_callback, 10)
        self.publisher_ = self.create_publisher(Imu, augmented_topic, 10)   
        
        self.curr_tick = self.prev_tick = self.get_clock().now().nanoseconds   #Time (nanoseconds)

        self.imu_aug = Imu()

    def listener_callback(self, msg):
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        tx = msg.angular_velocity.x
        ty = msg.angular_velocity.y
        tz = msg.angular_velocity.z
        # msg.header.stamp #bulitin.msg.Time (sec, nanosec)
        self.get_logger().info(f'Received @{msg.header.stamp.sec}:{msg.header.stamp.nanosec} [{msg.angular_velocity.x}, {msg.angular_velocity.y}, {msg.angular_velocity.z}]' )
        
        self.imu_aug.header = msg.header
        self.imu_aug.linear_acceleration.x = ax
        self.imu_aug.linear_acceleration.y = ay
        self.imu_aug.linear_acceleration.z = az
        self.imu_aug.angular_velocity.x = tx
        self.imu_aug.angular_velocity.y = ty
        self.imu_aug.angular_velocity.z = tz

        # calculate time elapse
        prev_tick = self.curr_tick
        self.curr_tick = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        delta_t = (self.curr_tick - prev_tick) / 10**9

        # publish augmented data
        self.publisher_.publish(self.imu_aug)
        self.get_logger().info(f"Publishing Augmented IMU Data to {augmented_topic} @ {msg.header.stamp.sec+msg.header.stamp.nanosec / 10**9}") 

def main(args=None):
    rclpy.init(args=args)

    augment_node = AugmentNode()
    rclpy.spin(augment_node)

    augment_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()