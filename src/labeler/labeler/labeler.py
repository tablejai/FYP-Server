from re import S
import rclpy
from rclpy.node import Node

from msgs.msg import ImuRawArray, ImuAugmented, ImuAugmentedArray    
from gesture_definition.gesture_definition import GestureDefinition
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
from pathlib import Path
from rosbags.typesys import get_types_from_idl, get_types_from_msg, register_types

class GW():
    def __init__(self):
        self.data = []
        self.class_ = GestureDefinition.NA

class Labeler(Node):
    
    def __init__(self):
        super().__init__('labeler')
        self.register_custom_types()

        # create reader instance and open for reading
        with Reader('/root/FYP-ROS/rosbag/rosbag2_2022_09_14-09_08_17') as reader:
            # topic and msgtype information is available on .connections list
            for connection in reader.connections:
                print("topic name: {0: <15}\t msg type: {1: <15}".format(connection.topic, connection.msgtype))

            # iterate over messages
            for connection, timestamp, rawdata in reader.messages():
                if connection.topic == '/ImuRawArray':
                    msg = deserialize_cdr(rawdata, connection.msgtype)
                    print(msg)

    def register_custom_types(self):
        msg_install_path = '/root/FYP-ROS/install/msgs/share/msgs/msg/'
        type_list = ["ImuRaw", "ImuRawArray", "ImuRawHeadless", "ImuAugmented", "ImuAugmentedArray", "ImuAugmentedHeadless"]
        add_types = {}
        for type_ in type_list:
            idl_text = Path(msg_install_path + type_ + '.idl').read_text()
            msg_text = Path(msg_install_path + type_ + '.msg').read_text()
            add_types.update(get_types_from_idl(idl_text))
            add_types.update(get_types_from_msg(msg_text, 'msgs/msg/' + type_))

        # make types available to rosbags serializers/deserializers
        register_types(add_types)

def main(args=None):
    rclpy.init(args=args)
    print(GestureDefinition.NA)

    labeler = Labeler()
    rclpy.spin(labeler)

    labeler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()