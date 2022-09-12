import rclpy
from rclpy.node import Node

from msgs.msg import ImuRawHeadless, ImuRawArray

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(ImuRawArray, 'ImuRawArray', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):

        # create msg
        msg = ImuRawArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        for _ in range(3):
            data = ImuRawHeadless()
            data.linear_acc_x, data.linear_acc_y, data.linear_acc_z = 0.1, 0.2, 0.3
            data.rotational_acc_x, data.rotational_acc_y, data.rotational_acc_z = 0.1, 0.2, 0.3
            msg.data.append(data)

        # publish
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing Pseudo Data @ {msg.header.stamp.sec+msg.header.stamp.nanosec / 10**9}") 

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()