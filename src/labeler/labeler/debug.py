import rclpy
from rclpy.node import Node

# msg filter
from message_filters import Subscriber, SimpleFilter, TimeSynchronizer, ApproximateTimeSynchronizer

# msg
from std_msgs.msg import String
from sensor_msgs.msg import Imu

class Labeler(Node):
    def __init__(self):
        super().__init__('Debug_Labeler')
        
        self.imu_msg_filter0 = Subscriber(self, Imu, '/Imu0')
        self.imu_msg_filter1 = Subscriber(self, Imu, '/Imu1')
        self.imu_msg_filter2 = Subscriber(self, Imu, '/Imu2')
        
        self.imu_msg_filter0.registerCallback(self.debug_cb, "Imu0")
        self.imu_msg_filter1.registerCallback(self.debug_cb, "Imu1")
        self.imu_msg_filter2.registerCallback(self.debug_cb, "Imu2")
        
        self.syncer = TimeSynchronizer([self.imu_msg_filter0, self.imu_msg_filter1, self.imu_msg_filter2], 100)
        self.syncer.registerCallback(self.sync_callback)

    def debug_cb(self, msg, name):
        self.get_logger().info(f"{name}:\t{msg.header.stamp.sec}.{msg.header.stamp.nanosec}")

    def sync_callback(self, msg0, msg1, msg2):
        self.get_logger().info(f"Synced @ {msg0.header.stamp.sec}.{msg0.header.stamp.nanosec}")

def main(args=None):
    rclpy.init(args=args)

    debug_labeler = Labeler()
    rclpy.spin(debug_labeler)

    debug_labeler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
