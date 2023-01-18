import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class Controller(Node):

    def __init__(self):
        super().__init__('Controller')

        # Create a subscriber
        self.subscriber = self.create_subscription(String, '/Command', self.callback, 10)
    
    def callback(self, msg):
        # TODO: call powerpoint api here
        self.get_logger().info('I heard: "%s"' % msg.data)

def main():
    rclpy.init()
    node = Controller()
    rclpy.spin(node)

    node.destroy_node() 
    rclpy.shutdown()

if __name__ == '__main__':
    main()