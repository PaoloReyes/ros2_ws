import rclpy

from rclpy.node import Node

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.get_logger().info('Line Follower Node started')

def main(args=None):
    rclpy.init()
    line_follower = LineFollower()
    rclpy.spin(line_follower)
    line_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()