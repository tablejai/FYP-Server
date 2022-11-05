import rclpy
from rclpy.node import Node

from flask import Flask, request
from flask_classful import FlaskView, route
import json

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

LENGTH = 3

class RawPublisher(Node):
    def __init__(self):
        rclpy.init(args=None)
        super().__init__('raw_publisher')

        self.publishers_ = []
        for i in range(LENGTH):
            self.publishers_.append(self.create_publisher(Imu, 'Imu_raw'+str(i), 10))

    def publishData(self, data):
        for i in range(LENGTH):
            self.publishers_[i].publish(data[i])
            acc = data[i].linear_acceleration
            vel = data[i].angular_velocity
            print(f'Publishing IMU[{i}]:\t[{acc.x}, {acc.y}, {acc.z}] [{vel.x}, {vel.y}, {vel.z}]')

class HelloView(FlaskView):
    app = Flask("Test")
    publisher= RawPublisher()

    @route('/',methods=['POST'], strict_slashes=False)
    def post(self):
        content = json.loads(request.data)
        
        data = []
        data_jsons = []
        for i in range(LENGTH):
            data_jsons.append(content["imu"+str(i)])

        for data_json in data_jsons:
            angular = Vector3()
            linear = Vector3()
            angular.x = float(data_json['ax'])
            angular.y = float(data_json['ay'])
            angular.z = float(data_json['az'])
            linear.x = float(data_json['tx'])
            linear.y = float(data_json['ty'])
            linear.z = float(data_json['tz'])
            imu = Imu()
            imu.angular_velocity = angular
            imu.linear_acceleration = linear
            data.append(imu)

        HelloView.publisher.publishData(data)
        
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

