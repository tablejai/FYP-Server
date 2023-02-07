import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from msgs.msg import Geasture
import pyautogui

class Commander(Node):
    def __init__(self):
        super().__init__('Commander')
        self.subscriber = self.create_subscription(Geasture, '/Geastures', self.callback, 10)
        self.get_logger.info(pyautogui.position())

    def callback(self, msg):
        if msg.type == Geasture.STATIC:    
            self.get_logger().info("STATIC")
        elif msg.type == Geasture.SLIDE_UP:
            self.get_logger().info("SLIDE_UP")
        elif msg.type == Geasture.SLIDE_DOWN:
            self.get_logger().info("SLIDE_DOWN")
        elif msg.type == Geasture.SLIDE_LEFT:
            self.get_logger().info("SLIDE_LEFT")
        elif msg.type == Geasture.SLIDE_RIGHT:
            self.get_logger().info("SLIDE_RIGHT")
        elif msg.type == Geasture.ZOOM_IN:
            self.get_logger().info("ZOOM_IN")
        elif msg.type == Geasture.ZOOM_OUT:
            self.get_logger().info("ZOOM_OUT")
        elif msg.type == Geasture.HIGHLIGHT:
            self.get_logger().info("HIGHLIGHT")
        elif msg.type == Geasture.ON_YES:
            self.get_logger().info("ON_YES")
        elif msg.type == Geasture.OFF_NO:
            self.get_logger().info("OFF_NO")
        elif msg.type == Geasture.END:
            self.get_logger().info("END")
        else:
            pass

def main(args=None):
    rclpy.init()
    node = Commander()
    rclpy.spin(node)

    node.destroy_node() 
    rclpy.shutdown()

if __name__ == '__main__':
    main()
