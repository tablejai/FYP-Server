import rclpy
from rclpy.node import Node

class Labeler(Node):

    def __init__(self):
        super().__init__('labeler')

def main(args=None):
    rclpy.init(args=args)

    labeler = Labeler()
    rclpy.spin(labeler)

    labeler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()