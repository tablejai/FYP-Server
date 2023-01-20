import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from msgs.msg import Geasture

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import message_filters

class Detector(Node):
    def __init__(self):
        super().__init__('Detector')

        # create subscribers
        self.imu_subscribers = []
        self.imu_mag_subscribers = []
        for i in range(3):
            self.imu_subscribers.append(message_filters.Subscriber(self, Imu, '/Imu'+str(i)))
            self.imu_mag_subscribers.append(message_filters.Subscriber(self, MagneticField, '/Imu'+str(i)+'/Mag'))
        
        self.syncer = message_filters.TimeSynchronizer([fs for fs in self.imu_subscribers + self.imu_mag_subscribers], 10)
        self.syncer.registerCallback(self.callback)

        # create a transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # create a publisher
        self.command_publisher = self.create_publisher(Geasture, '/Geastures', 10)
    
    def callback(self, *args):
        pass
        # self.command_publisher.publish(String(data='hello'))

def main():
    rclpy.init()
    node = Detector()
    rclpy.spin(node)

    node.destroy_node() 
    rclpy.shutdown()

if __name__ == '__main__':
    main()
