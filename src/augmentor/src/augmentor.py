import rclpy
from rclpy.node import Node
from msgs.msg import ImuRawArray, ImuAugmentedHeadless, ImuAugmentedArray       

class AugmentNode(Node):

    def __init__(self):
        super().__init__('imu_subscriber')
        
        self.subscription = self.create_subscription(ImuRawArray, 'ImuRawArray', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(ImuAugmentedArray, 'ImuAugmentedArray', 10)   
        
        self.curr_tick = self.prev_tick = self.get_clock().now().nanoseconds   #Time (nanoseconds)

        self.imu_aug = ImuAugmentedArray()
        for _ in range(3):
            self.imu_aug.data.append(ImuAugmentedHeadless())

    def listener_callback(self, msg):
        # msg.header.stamp #bulitin.msg.Time (sec, nanosec)
        # self.get_logger().info(f'recv @{msg.header.stamp.sec}:{msg.header.stamp.nanosec} [{msg.linear_acc[0]}, {msg.linear_acc[1]}, {msg.linear_acc[2]}]' )
        
        # calculate time elapse
        self.prev_tick = self.curr_tick
        self.curr_tick = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        delta_t = (self.curr_tick - self.prev_tick) / 10**9

        # integration
        self.imu_aug.header = msg.header
        for imu in msg.data:
            for i in range(3):
                # translation
                self.imu_aug.data[i].linear_trans_x += self.imu_aug.data[i].linear_vel_x * delta_t
                self.imu_aug.data[i].linear_trans_y += self.imu_aug.data[i].linear_vel_y * delta_t
                self.imu_aug.data[i].linear_trans_z += self.imu_aug.data[i].linear_vel_z * delta_t

                self.imu_aug.data[i].linear_vel_x += imu.linear_acc_x * delta_t
                self.imu_aug.data[i].linear_vel_y += imu.linear_acc_y * delta_t
                self.imu_aug.data[i].linear_vel_z += imu.linear_acc_z * delta_t
                 
                self.imu_aug.data[i].linear_acc_x = imu.linear_acc_x
                self.imu_aug.data[i].linear_acc_y = imu.linear_acc_y
                self.imu_aug.data[i].linear_acc_z = imu.linear_acc_z

                # rotation
                self.imu_aug.data[i].rotational_trans_x += self.imu_aug.data[i].rotational_vel_x * delta_t
                self.imu_aug.data[i].rotational_trans_y += self.imu_aug.data[i].rotational_vel_y * delta_t
                self.imu_aug.data[i].rotational_trans_z += self.imu_aug.data[i].rotational_vel_z * delta_t

                self.imu_aug.data[i].rotational_vel_x += imu.rotational_acc_x * delta_t
                self.imu_aug.data[i].rotational_vel_y += imu.rotational_acc_y * delta_t
                self.imu_aug.data[i].rotational_vel_z += imu.rotational_acc_z * delta_t
                 
                self.imu_aug.data[i].rotational_acc_x = imu.rotational_acc_x
                self.imu_aug.data[i].rotational_acc_y = imu.rotational_acc_y
                self.imu_aug.data[i].rotational_acc_z = imu.rotational_acc_z
    
        # publish augmented data
        self.publisher_.publish(self.imu_aug)

def main(args=None):
    rclpy.init(args=args)

    augment_node = AugmentNode()
    rclpy.spin(augment_node)

    augment_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()