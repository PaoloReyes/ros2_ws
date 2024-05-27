import rclpy # type: ignore
import numpy as np

from rclpy.node import Node # type: ignore

from nav_msgs.msg import Odometry # type: ignore
from geometry_msgs.msg import Pose2D # type: ignore
from geometry_msgs.msg import Twist # type: ignore
from sensor_msgs.msg import LaserScan # type: ignore

from .submodules import units_utils as units

class HardSwitchControl(Node):
    def __init__(self):
        super().__init__('hard_switch_control')

        ## Parameters ##
        self.declare_parameters(
            namespace='',
            parameters=[
                ('lidar_linear_kp', rclpy.parameter.Parameter.Type.DOUBLE),
                ('lidar_angular_kp', rclpy.parameter.Parameter.Type.DOUBLE),
            ]
        )

        ## Subscribers ##
        self.create_subscription(LaserScan, '/lidar', self.lidar_callback, 10)

        ## Publishers ##
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def lidar_callback(self, msg):
        ranges = msg.ranges
        points = np.array(ranges)
        points[np.isinf(points)] = msg.range_max
        lower_valid_index = int(((-np.pi/2)-msg.angle_min)//msg.angle_increment)
        upper_valid_index = int(((np.pi/2)-msg.angle_min)//msg.angle_increment)
        points = points[lower_valid_index:upper_valid_index]
        points = np.array([units.polar_to_cartesian(point, idx*msg.angle_increment-np.pi/2) for idx, point in enumerate(points)])
        vector = [0., 0.]
        for point in points:
            vector[0] += point[0]
            vector[1] += point[1]
        vector = np.array(vector)
        vector = units.cartesian_to_polar(vector)
        vector_angle = vector[1]
        vector_distance = vector[0]

        cmd_vel = Twist()
        cmd_vel.linear.x = self.get_parameter('lidar_linear_kp').get_parameter_value().double_value*vector_distance
        cmd_vel.angular.z = self.get_parameter('lidar_angular_kp').get_parameter_value().double_value*vector_angle
        self.cmd_vel_publisher.publish(cmd_vel)

def main():
    rclpy.init()
    hard_switch_control = HardSwitchControl()
    rclpy.spin(hard_switch_control)
    hard_switch_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
