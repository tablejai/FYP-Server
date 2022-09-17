import rclpy
from rclpy.node import Node

from msgs.msg import ImuRawArray, ImuAugmented, ImuAugmentedArray    
from gesture_definition.gesture_definition import GestureDefinition
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
from pathlib import Path
from rosbags.typesys import get_types_from_idl, get_types_from_msg, register_types
from std_msgs.msg import String

# threading
import threading
import time

bag_path = '/root/FYP-ROS/rosbag/rosbag2_2022_09_16-08_00_02'

class Labeler(Node):

    def __init__(self):
        super().__init__('Labeler')
        self.register_custom_types()
        self.publisher_ = self.create_publisher(String, 'label', 10)
        self.timer_ = None
        self.gw = []
        threading.Thread(target=self.start_labeling).start()

    def start_labeling(self):
        # create reader instance and open for reading
        raw_count = 0
        with Reader(bag_path) as reader:
            # topic and msgtype information is available on .connections list
            for connection in reader.connections:
                print("topic name: {0: <15}\t count {1: <15} msg type: {2: <15}".format(connection.topic, connection.msgcount, connection.msgtype))
                if(connection.topic == "/ImuAugmentedArray"):
                    raw_count = connection.msgcount

            buffer = []
            counter = 0
            # iterate over messages
            for connection, timestamp, rawdata in reader.messages():
                if connection.topic == '/ImuAugmentedArray':
                    msg = deserialize_cdr(rawdata, connection.msgtype)
                    counter+=1
                    if msg.is_eng.data == True:
                        print("============END OF GESTURE============\n")
                        self.timer = self.create_timer(1, self.timer_callback)
                        prompt = "0\tNA\n1\tSlideLeft\n2\tSlideRight\n3\tSlideUp\n4\tSlideDown\n5\tZoomIn\n6\tZoomOut\n7\tHightlight\n8\tOn\n9\tOff\nPlease label the gesture: "
                        self.gw.append(int(input(prompt)))
                        self.timer.cancel()
                        buffer.clear()
                        print("\n\n=================data=================")
                    else:
                        buffer.append(msg.data)
                        print('|{0:3d} - [{1:7.3f} {2:7.3f} {3:7.3f} {4:7.3f}]'.format(len(buffer), \
                                                                                        msg.data[0].quaternion_x, \
                                                                                        msg.data[0].quaternion_y, \
                                                                                        msg.data[0].quaternion_z, \
                                                                                        msg.data[0].quaternion_w))
                        if(counter == raw_count):
                            print("end of file")
                            
            # save labels to file

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
    
    def timer_callback(self):
        # self.get_logger().info("timer callback")
        msg = String()
        msg.data = "callback"
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    labeler = Labeler()
    rclpy.spin(labeler)

    labeler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


##
## msg definition
##
# msgs__msg__ImuRawArray(
#     header=std_msgs__msg__Header(
#         stamp=builtin_interfaces__msg__Time(sec=1663146500, nanosec=167614138, __msgtype__='builtin_interfaces/msg/Time'), 
#         frame_id='', 
#         __msgtype__='std_msgs/msg/Header'
#     ), 
#     data=[
#         msgs__msg__ImuRawHeadless(linear_acc_x=0.1, linear_acc_y=0.2, linear_acc_z=0.3, rotational_acc_x=0.1, rotational_acc_y=0.2, rotational_acc_z=0.3, __msgtype__='msgs/msg/ImuRawHeadless'),
#         msgs__msg__ImuRawHeadless(linear_acc_x=0.1, linear_acc_y=0.2, linear_acc_z=0.3, rotational_acc_x=0.1, rotational_acc_y=0.2, rotational_acc_z=0.3, __msgtype__='msgs/msg/ImuRawHeadless'), 
#         msgs__msg__ImuRawHeadless(linear_acc_x=0.1, linear_acc_y=0.2, linear_acc_z=0.3, rotational_acc_x=0.1, rotational_acc_y=0.2, rotational_acc_z=0.3, __msgtype__='msgs/msg/ImuRawHeadless')
#     ],
#     __msgtype__='msgs/msg/ImuRawArray'
# )