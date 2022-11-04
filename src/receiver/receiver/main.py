import rclpy
import json
from rclpy.node import Node

from std_msgs.msg import String

from flask import Flask, request, jsonify
from flask_classful import FlaskView, route

class MinimalPublisher(Node):
    def __init__(self):
        rclpy.init(args=None)
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'raw', 10)

    def publishData(self, data):
        msg = String()
        msg.data = data
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

class HelloView(FlaskView):
    app = Flask("Test")
    publisher= MinimalPublisher()

    @route('/',methods=['POST'], strict_slashes=False)
    def post(self):
        content = json.loads(request.data)
        imu0 = content["imu0"]
        imu0_ax = imu0['ax']
        imu0_ay = imu0['ay']
        imu0_az = imu0['az']
        HelloView.publisher.publishData(str(imu0_ax))
        return 'Hi'

def main(args=None):
    HelloView.register(HelloView.app)
    HelloView.app.run(host='0.0.0.0', port=5001, debug=True)


    rclpy.spin(HelloView.publisher)

    HelloView.publisher.init()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

