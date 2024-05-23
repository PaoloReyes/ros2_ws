import rclpy

from rclpy.node import Node

class LineFollower(Node):
    def __init__(self):
        pass

def main(args=None):
    rclpy.init()
    line_follower = LineFollower()
    rclpy.spin(line_follower)
    line_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()