import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu

class PesudoPublisher(Node):
    def __init__(self):
        super().__init__('pesudo_imu_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu/data_raw', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):

        data = Imu()
        data.header.stamp = self.get_clock().now().to_msg()
        data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z = 0.1, 0.2, 0.3
        data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z = 0.1, 0.2, 0.3

        # publish
        self.publisher_.publish(data)
        self.get_logger().info("Publishing Pseudo Data @ {0: f}".format(data.header.stamp.sec+data.header.stamp.nanosec / 10**9)) 

def main(args=None):
    rclpy.init(args=args)

    pesudo_publisher = PesudoPublisher()
    rclpy.spin(pesudo_publisher)

    pesudo_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()