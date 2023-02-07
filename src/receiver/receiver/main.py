import rclpy
from rclpy.node import Node

import threading 
from flask import Flask, request
import json

from std_msgs.msg import Header
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import Vector3
from builtin_interfaces.msg import Time

IMU_NODE_NUM = 3
ASSIGN_TIMESTAMP_BY_ROS = True

app = Flask(__name__)
rclpy.init()

import logging
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

class RawPublisherMaster(Node):
    def __init__(self):
        super().__init__('Receiver')

        self.imu_publishers_ = []
        self.mag_publishers_ = []
        for i in range(IMU_NODE_NUM):
            self.imu_publishers_.append(self.create_publisher(Imu, '/Imu'+str(i)+'/Raw', 10))
            self.mag_publishers_.append(self.create_publisher(MagneticField, '/Imu'+str(i)+'/Mag', 10))

    def publish_data(self, data):
        for i in range(IMU_NODE_NUM):
            stamp, imu, magField = data[i]

            # publish data
            self.imu_publishers_[i].publish(imu)
            self.mag_publishers_[i].publish(magField)

            # Print log
            accel = imu.linear_acceleration
            gyro = imu.angular_velocity
            mag = magField.magnetic_field
            self.get_logger().info(f'Publishing IMU[{i:2d}]: @{stamp.sec:10d},{stamp.nanosec:9d} A[{accel.x:+6.2f}, {accel.y:+6.2f}, {accel.z:+6.2f}] G[{gyro.x:+6.2f}, {gyro.y:+6.2f}, {gyro.z:+6.2f}] M[{mag.x:+6.2f}, {mag.y:+6.2f}, {mag.z:+6.2f}]')

publisherMaster = RawPublisherMaster()

@app.route('/')
def publish_imu_data():
    content = {}
    try:
        request.data = request.data.split(b'\x00')[0]
        content = json.loads(request.data)
        data = []
        if ASSIGN_TIMESTAMP_BY_ROS:
            t_ = publisherMaster.get_clock().now().nanoseconds
            sec, nsec = t_ // 10**9, t_ % 10**9
        else:
            sec, nsec = int(content['t_sec']), int(content['t_nanosec'])
        for i in range(IMU_NODE_NUM):
            imu_raw = content["imu"+str(i)]
            accel = Vector3(x=float(imu_raw['ax']), y=float(imu_raw['ay']), z=float(imu_raw['az']))
            gyro = Vector3(x=float(imu_raw['gx']), y=float(imu_raw['gy']), z=float(imu_raw['gz']))
            mag = Vector3(x=float(imu_raw['mx']), y=float(imu_raw['my']), z=float(imu_raw['mz']))

            stamp = Time(sec=sec, nanosec=nsec)
            header = Header(stamp=stamp, frame_id='imu'+str(i))
            imu = Imu(header=header, linear_acceleration=accel, angular_velocity=gyro)
            magField = MagneticField(header=header, magnetic_field=mag)
            data.append((stamp, imu, magField))

        publisherMaster.publish_data(data)
        return "0"

    except json.JSONDecodeError as e:
        print(request.data)
        return "-1"

# keep this order

app.run(host='0.0.0.0', port=5001, debug=True)
rclpy.spin(publisherMaster)

# destroy node and shutdown
publisherMaster.destroy_node()
rclpy.shutdown()
