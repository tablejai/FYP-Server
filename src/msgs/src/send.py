import rclpy
from rclpy.node import Node

from msgs.msg import ImuRaw   


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(ImuRaw, 'IMU_test', 10)    
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = ImuRaw()                     
        msg.header.stamp = self.get_clock().now().to_msg()    
        msg.linear_acc = [0.1, 0.2, 0.3]                                     
        msg.rotational_acc = [0.1, 0.2, 0.2]                                     
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: [{msg.linear_acc[0]}, {msg.linear_acc[1]}, {msg.linear_acc[2]}]') 
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()