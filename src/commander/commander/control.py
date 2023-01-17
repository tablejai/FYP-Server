import rclpy
from rclpy.node import Node

class Controller(Node):

    def __init__(self):
        super().__init__('Controller')

def main():
    rclpy.init()
    node = Controller()
    rclpy.spin(node)

    node.destroy_node() 
    rclpy.shutdown()

if __name__ == '__main__':
    main()
