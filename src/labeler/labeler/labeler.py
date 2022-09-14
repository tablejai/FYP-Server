import rclpy
from rclpy.node import Node

from msgs.msg import ImuRawArray, ImuAugmented, ImuAugmentedArray    
from gesture_definition.gesture_definition import GestureDefinition
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr

class GW():
    def __init__(self):
        self.data = []
        self.class_ = GestureDefinition.NA

class Labeler(Node):
    
    def __init__(self):
        super().__init__('labeler')
        self.gw = GW()

        # create reader instance and open for reading
        with Reader('/root/FYP-ROS/rosbag/rosbag2_2022_09_14-09_08_17') as reader:
            # topic and msgtype information is available on .connections list
            for connection in reader.connections:
                print(connection.topic, connection.msgtype)

            # iterate over messages
            for connection, timestamp, rawdata in reader.messages():
                if connection.topic == '/imu_raw/Imu':
                    msg = deserialize_cdr(rawdata, connection.msgtype)
                    print(msg.header.frame_id)
    
    # def listener_callback(self, msg):
    #     if msg.is_eng:
    #         if self.gw.data.size() == 0:
    #             pass
    #         else:
    #             class_ = input("Input Gesture Class and Press Enter to continue...")
    #             self.gw.class_ = class_
    #             self.gw.data = []

    #     self.gw.data.append(msg.data)
        
    #     # save gesture to file


def main(args=None):
    rclpy.init(args=args)
    print(GestureDefinition.NA)

    labeler = Labeler()
    rclpy.spin(labeler)

    labeler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()