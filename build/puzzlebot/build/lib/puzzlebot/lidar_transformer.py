import rclpy

from rclpy.node import Node

from sensor_msgs.msg import LaserScan

class LidarTransformer(Node):
    def __init__(self):
        super().__init__('lidar_transformer')

        self.create_subscription(LaserScan, '/raw_lidar', self.lidar_callback, 10)

        self.lidar_publisher = self.create_publisher(LaserScan, '/lidar', 10)

    def lidar_callback(self, msg):
        msg.header.frame_id = 'lidar_link'
        self.lidar_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    lidar_transformer = LidarTransformer()
    rclpy.spin(lidar_transformer)
    lidar_transformer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()