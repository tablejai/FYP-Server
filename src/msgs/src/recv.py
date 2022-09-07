import rclpy
from rclpy.node import Node

from msgs.msg import ImuRaw        


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(ImuRaw, 'ImuRaw', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: [{msg.linear_acc[0]}, {msg.linear_acc[1]}, {msg.linear_acc[2]}]' ) 

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()