import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from msgs.msg import ImuCluster

class PesudoPublisher(Node):

    def __init__(self):
        super().__init__('pesudo_imu_publisher')
        self.publisher_ = self.create_publisher(ImuCluster, 'ImuCluster', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):

        # create msg
        msg = ImuCluster()
        msg.header.stamp = self.get_clock().now().to_msg()
        for _ in range(3):
            data = Imu()
            # data.header = msg.header
            data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z = 0.1, 0.2, 0.3
            data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z = 0.1, 0.2, 0.3
            msg.data.append(data)

        # publish
        self.publisher_.publish(msg)
        self.get_logger().info("Publishing Pseudo Data @ {0: f}".format(msg.header.stamp.sec+msg.header.stamp.nanosec / 10**9)) 

def main(args=None):
    rclpy.init(args=args)

    pesudo_publisher = PesudoPublisher()
    rclpy.spin(pesudo_publisher)

    pesudo_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()