import rclpy
from rclpy.node import Node

from flask import Flask, request

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import json

LENGTH = 3

class RawPublisher(Node):
    def __init__(self):
        rclpy.init(args=None)
        super().__init__('raw_publisher')

        self.publishers_ = []
        for i in range(LENGTH):
            self.publishers_.append(self.create_publisher(Imu, 'Imu_raw'+str(i), 10))

    def publish_data(self, data):
        for i in range(LENGTH):
            self.publishers_[i].publish(data[i])
            acc = data[i].linear_acceleration
            vel = data[i].angular_velocity
            print(f'Publishing IMU[{i}]:\t[{acc.x}, {acc.y}, {acc.z}] [{vel.x}, {vel.y}, {vel.z}]')

app = Flask(__name__)
publisher = RawPublisher()

@app.route('/', methods=['POST'])
def publish_imu_data():
    content = json.loads(request.data)
    data = []
    data_jsons = []
    for i in range(LENGTH):
        data_jsons.append(content["imu"+str(i)])

    for data_json in data_jsons:
        accel = Vector3()
        gyro = Vector3()
        mag = Vector3()
        accel.x = float(data_json['ax'])
        accel.y = float(data_json['ay'])
        accel.z = float(data_json['az'])
        gyro.x = float(data_json['gx'])
        gyro.y = float(data_json['gy'])
        gyro.z = float(data_json['gz'])
        mag.x = float(data_json['mx'])
        mag.y = float(data_json['my'])
        mag.z = float(data_json['mz'])

        imu = Imu()
        imu.linear_acceleration = accel
        imu.angular_velocity = gyro
        data.append(imu)

    publisher.publish_data(data)

    return 'Data Received and Published!'

def main(args=None):
    app.run(host='0.0.0.0', port=5001, debug=True)
    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()