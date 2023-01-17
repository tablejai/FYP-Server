import rclpy
from rclpy.node import Node

class Detector(Node):
    def __init__(self):
        super().__init__('Detector')

def main():
    rclpy.init()
    node = Detector()
    rclpy.spin(node)

    node.destroy_node() 
    rclpy.shutdown()

if __name__ == '__main__':
    main()
