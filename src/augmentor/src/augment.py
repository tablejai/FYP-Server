import rclpy
from rclpy.node import Node
from msgs.msg import ImuRaw, ImuAugmented       

class AugmentNode(Node):

    def __init__(self):
        super().__init__('imu_subscriber')
        
        self.subscription = self.create_subscription(ImuRaw, 'ImuRaw', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(ImuAugmented, 'ImuAugmented', 10)   
        
        self.curr_tick = self.prev_tick = self.get_clock().now().nanoseconds   #Time (nanoseconds)

        self.imu_augmented = ImuAugmented()
        self.imu_augmented.linear_trans = [0.0, 0.0, 0.0]
        self.imu_augmented.linear_vel = [0.0, 0.0, 0.0]
        self.imu_augmented.linear_acc = [0.0, 0.0, 0.0]

    def listener_callback(self, msg):
        # msg.header.stamp #bulitin.msg.Time (sec, nanosec)
        # self.get_logger().info(f'recv @{msg.header.stamp.sec}:{msg.header.stamp.nanosec} [{msg.linear_acc[0]}, {msg.linear_acc[1]}, {msg.linear_acc[2]}]' )
        
        # calculate time elapse
        self.prev_tick = self.curr_tick
        self.curr_tick = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        delta_t = (self.curr_tick - self.prev_tick) / 10**9

        # integration
        self.imu_augmented.header = msg.header
        for i in range(3):
            self.imu_augmented.linear_trans[i] += self.imu_augmented.linear_vel[i] * delta_t 
            self.imu_augmented.linear_vel[i] += msg.linear_acc[i] * delta_t 
            self.imu_augmented.linear_acc[i] = msg.linear_acc[i]
        
        print("trans", self.imu_augmented.linear_trans)
        print("veloc", self.imu_augmented.linear_vel)
        print("accel", self.imu_augmented.linear_acc)

        # publish augmented data
        self.publisher_.publish(self.imu_augmented)

def main(args=None):
    rclpy.init(args=args)

    augment_node = AugmentNode()
    rclpy.spin(augment_node)

    augment_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()