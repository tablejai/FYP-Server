import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import socket
import json


class Publisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)

    def publish(self, msg):
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


class SocketReceiver():

    def __init__(self):
        self.publisher = Publisher()
        self.host = "0.0.0.0"
        self.port = 5001
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((self.host, self.port))
        self.socket.listen(5)
        print('server start at: %s:%s' % (self.host, self.port))
        print('wait for connection...')

    def run(self):
        while True:
            conn, addr = self.socket.accept()
            print('connected by ' + str(addr))

            while True:
                indata = conn.recv(65525)
                if len(indata) == 0:
                    conn.close()
                    print('client closed connection.')
                    break
                decoded_indata = indata.decode()
                json_indata = json.loads(decoded_indata)
                print('recv: ' + json_indata)
                # TODO: Make this publisher capable of accepting json data in the dedicated data type
                self.publisher.publish(decoded_indata)


def main(args=None):
    rclpy.init(args=args)

    receiver = SocketReceiver()

    rclpy.spin(receiver.publisher)

    receiver.run()

    # TODO: Look into whether these two things are acutally need lmao
    receiver.publisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
