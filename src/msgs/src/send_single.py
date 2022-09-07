import rclpy
from rclpy.node import Node

from msgs.msg import ImuRaw   


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(ImuRaw, 'ImuRaw', 10)    
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # create msg
        msg = ImuRaw()                     
        msg.header.stamp = self.get_clock().now().to_msg()    
        msg.linear_acc_x, msg.linear_acc_y, msg.linear_acc_z = 0.1, 0.2, 0.3
        msg.rotational_acc_x, msg.rotational_acc_y, msg.rotational_acc_z = 0.1, 0.2, 0.3  

        # publish
        self.publisher_.publish(msg)

        # log and increment
        self.get_logger().info(f'Publishing: [{msg.linear_acc_x}, {msg.linear_acc_y}, {msg.linear_acc_z}]') 

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()