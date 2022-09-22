import rclpy
from rclpy.node import Node

from msgs.msg import ImuRaw, ImuRawArray  


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(ImuRawArray, 'ImuRawArray', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: [{msg.data[0].linear_acc_x}, {msg.data[0].linear_acc_y}, {msg.data[0].linear_acc_z}]' ) 

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()