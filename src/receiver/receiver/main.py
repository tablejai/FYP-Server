import rclpy
from rclpy.node import Node

from flask import Flask, request

from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import Vector3
import json

IMU_NODE_NUM = 3

app = Flask(__name__)
rclpy.init()

class RawPublisherMaster(Node):
    def __init__(self):
        super().__init__('raw_publisher')

        self.imu_publishers_ = []
        self.mag_publishers_ = []
        for i in range(IMU_NODE_NUM):
            self.imu_publishers_.append(self.create_publisher(Imu, 'Imu_raw'+str(i), 10))
            self.mag_publishers_.append(self.create_publisher(MagneticField, 'Mag_raw'+str(i), 10))

    def publish_data(self, data):
        for i in range(IMU_NODE_NUM):
            imu, magField = data[i]

            # publish data
            self.imu_publishers_[i].publish(imu)
            self.mag_publishers_[i].publish(magField)

            # Print log
            accel = imu.linear_acceleration
            gyro = imu.angular_velocity
            mag = magField.magnetic_field
            print(f'Publishing IMU[{i:2d}]: A[{accel.x:+8.2f}, {accel.y:+8.2f}, {accel.z:+8.2f}] G[{gyro.x:+8.2f}, {gyro.y:+8.2f}, {gyro.z:+8.2f}] M[{mag.x:+8.2f}, {mag.y:+8.2f}, {mag.z:+8.2f}]')

@app.route('/', methods=['POST'])
def publish_imu_data():
    content = json.loads(request.data)
    data = []
    for i in range(IMU_NODE_NUM):
        data_json = content["imu"+str(i)]
        accel = Vector3(x=float(data_json['ax']), y=float(data_json['ay']), z=float(data_json['az']))
        gyro = Vector3(x=float(data_json['gx']), y=float(data_json['gy']), z=float(data_json['gz']))
        mag = Vector3(x=float(data_json['mx']), y=float(data_json['my']), z=float(data_json['mz']))

        imu = Imu(linear_acceleration=accel, angular_velocity=gyro)
        magField = MagneticField(magnetic_field=mag)
        data.append((imu, magField))

    publisherMaster.publish_data(data)
    return 'Data Received and Published!'

# keep this order
publisherMaster = RawPublisherMaster()

app.run(host='0.0.0.0', port=5001, debug=True)
rclpy.spin(publisherMaster)

# destroy node and shutdown
publisherMaster.destroy_node()
rclpy.shutdown()