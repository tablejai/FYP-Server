import rclpy
from rclpy.node import Node

from msgs.msg import ImuRawArray, ImuAugmented, ImuAugmentedArray, ImuAugmentedHeadless 
from gesture_definition.gesture_definition import GestureDefinition
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
from pathlib import Path
from rosbags.typesys import get_types_from_idl, get_types_from_msg, register_types

# threading
import threading
import time

bag_name = 'rosbag2_2022_09_17-16_09_14'
bag_path = '/root/FYP-ROS/rosbag/bag/' + bag_name
data_path = '/root/FYP-ROS/rosbag/data/' + bag_name + ".txt"
label_path = '/root/FYP-ROS/rosbag/label/' + bag_name + ".txt"
republishing_time = 0.3

class Labeler(Node):

    def __init__(self):
        super().__init__('Labeler')
        self.register_custom_types()
        
        self.publisher_ = self.create_publisher(ImuAugmentedArray, '/Imu_viz', 10)
        self.timer_ = None
        
        self.labels = []
        self.buffer = []

        self.label_thread = threading.Thread(target=self.start_labeling)
        self.label_thread.start()

    def start_labeling(self):
        data_f = open(data_path, 'w')
        label_f = open(label_path, 'w')
        ttl_raw_count = 0
        # create reader instance and open for reading
        with Reader(bag_path) as reader:
            # topic and msgtype information is available on .connections list
            for connection in reader.connections:
                print("topic name: {0: <15}\t count {1: <15} msg type: {2: <15}".format(connection.topic, connection.msgcount, connection.msgtype))
                if(connection.topic == "/ImuAugmentedArray"):
                    ttl_raw_count = connection.msgcount

            counter = 0
            print("\n\n===============data[{0:4d}/{1:4d}]===============".format(counter, ttl_raw_count))
            
            # iterate over messages
            for connection, timestamp, rawdata in reader.messages():
                if connection.topic == '/ImuAugmentedArray':
                    msg = deserialize_cdr(rawdata, connection.msgtype)
                    counter+=1
                    if msg.is_eng.data == True:
                        print("==============END OF GESTURE===============\n")
                        self.buffer.append(msg)

                        # start publishing 
                        self.timer = self.create_timer(1, self.timer_callback)

                        # create prompt and wait for user input
                        prompt = ""
                        for gd in GestureDefinition:
                            prompt += str(gd.value) + "\t" + gd.name + "\n"
                        prompt += "Please label the gesture: "

                        # validate user input
                        input_ = input(prompt)
                        while not input_.isnumeric():
                            self.get_logger().warn("input must be a number")
                            input_ = input(prompt)
                        while int(input_) > len(GestureDefinition) or int(input_) < 0:
                            self.get_logger().warn("input must be between 0 and " + str(len(GestureDefinition)-1))
                            input_ = input(prompt)
                        label = int(input_)

                        # collect label
                        self.labels.append(label)

                        # stop publishing 
                        self.timer.cancel()
                        
                        # save data to file
                        for data in self.buffer:
                            data_f.write(data.__str__() + '\n')
                        label_f.write(str(label) + '\n')
                        data_f.write("==============END OF GESTURE===============\n")

                        self.buffer.clear()
                        print("\n\n===============data[{0:4d}/{1:4d}]===============".format(counter, ttl_raw_count))
                    else:
                        # add data to buffer
                        self.buffer.append(msg)
                        print('|{0:3d}|   n0[{1:6.2f},{2:6.2f},{3:6.2f},{4:6.2f}]   n1[{5:6.2f},{6:6.2f},{7:6.2f},{8:6.2f}]   n2[{9:6.2f},{10:6.2f},{11:6.2f},{12:6.2f}]'\
                            .format( \
                                len(self.buffer), \
                                msg.data[0].quaternion_x, msg.data[0].quaternion_y, msg.data[0].quaternion_z, msg.data[0].quaternion_w, \
                                msg.data[1].quaternion_x, msg.data[1].quaternion_y, msg.data[1].quaternion_z, msg.data[1].quaternion_w, \
                                msg.data[2].quaternion_x, msg.data[2].quaternion_y, msg.data[2].quaternion_z, msg.data[2].quaternion_w, \
                            ))
            
            # close file streams
            self.get_logger().info("end of file")
            self.get_logger().info("closing file streams")
            data_f.close()
            label_f.close()

            # stop thread and safely exit
            self.timer.cancel()
            # TODO

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
        for msg in self.buffer:
            msg_pub = ImuAugmentedArray()
            for in_ in msg.data:
                out_ = ImuAugmentedHeadless()
                out_.linear_acc_x = in_.linear_acc_x
                out_.linear_acc_y = in_.linear_acc_y
                out_.linear_acc_z = in_.linear_acc_z

                out_.linear_vel_x = in_.linear_vel_x
                out_.linear_vel_y = in_.linear_vel_y
                out_.linear_vel_z = in_.linear_vel_z

                out_.linear_trans_x = in_.linear_trans_x
                out_.linear_trans_y = in_.linear_trans_y
                out_.linear_trans_z = in_.linear_trans_z

                out_.rotational_acc_x = in_.rotational_acc_x
                out_.rotational_acc_y = in_.rotational_acc_y
                out_.rotational_acc_z = in_.rotational_acc_z

                out_.rotational_vel_x = in_.rotational_vel_x
                out_.rotational_vel_y = in_.rotational_vel_y
                out_.rotational_vel_z = in_.rotational_vel_z

                out_.rotational_trans_x = in_.rotational_trans_x
                out_.rotational_trans_y = in_.rotational_trans_y
                out_.rotational_trans_z = in_.rotational_trans_z

                out_.quaternion_x = in_.quaternion_x
                out_.quaternion_y = in_.quaternion_y
                out_.quaternion_z = in_.quaternion_z
                out_.quaternion_w = in_.quaternion_w
                
                msg_pub.data.append(out_)

            msg_pub.header.stamp.sec = msg.header.stamp.sec
            msg_pub.header.stamp.nanosec = msg.header.stamp.nanosec
            msg_pub.is_eng.data = msg.is_eng.data

            self.publisher_.publish(msg_pub)
            time.sleep(republishing_time)

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