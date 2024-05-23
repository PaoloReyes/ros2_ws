import rclpy
import numpy as np

from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist

from .submodules import units_utils as units

def get_z_rotation_from_quaternion(x, y, z, w):
    rotation_z = np.arctan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))
    return rotation_z

class HardSwitchControl(Node):
    def __init__(self):
        super().__init__('hard_switch_control')

        ## Parameters ##
        self.declare_parameters(
            namespace='',
            parameters=[
                ('angle_range', rclpy.parameter.Parameter.Type.DOUBLE),
                ('tolerance_distance', rclpy.parameter.Parameter.Type.DOUBLE),
                ('linear_kp', rclpy.parameter.Parameter.Type.DOUBLE),
                ('angular_kp', rclpy.parameter.Parameter.Type.DOUBLE),
                ('angle_range', rclpy.parameter.Parameter.Type.DOUBLE),
                ('linear_acc', rclpy.parameter.Parameter.Type.DOUBLE),
                ('angular_acc', rclpy.parameter.Parameter.Type.DOUBLE),
            ]
        )

        try:
            self.get_parameters(
                ['angle_range',
                'tolerance_distance',
                'linear_kp',
                'angular_kp',
                'angle_range',
                'linear_acc',
                'angular_acc']
            )
        except:
            self.set_parameters([rclpy.parameter.Parameter('tolerance_distance', rclpy.parameter.Parameter.Type.DOUBLE, 0.1),
                                 rclpy.parameter.Parameter('angle_range', rclpy.parameter.Parameter.Type.DOUBLE, np.pi/2),
                                 rclpy.parameter.Parameter('linear_kp', rclpy.parameter.Parameter.Type.DOUBLE, 0.5),
                                 rclpy.parameter.Parameter('angular_kp', rclpy.parameter.Parameter.Type.DOUBLE, 1.5),
                                 rclpy.parameter.Parameter('linear_acc', rclpy.parameter.Parameter.Type.DOUBLE, 0.05),
                                 rclpy.parameter.Parameter('angular_acc', rclpy.parameter.Parameter.Type.DOUBLE, 0.3)])

        ## Variables ##
        self.position = [0, 0]
        self.orientation = 0
        self.distance_error = 0
        self.orientation_error = 0
        self.goal = [0, 0]
        self.cmd_vel = Twist()

        ## Subscribers ##
        self.create_subscription(Odometry, '/odometry', self.odometry_callback, 10)
        self.create_subscription(Pose2D, '/goal', self.goal_callback, 10)

        ## Publishers ##
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        ## Timers ##
        TIMER_PERIOD = 0.1
        self.create_timer(TIMER_PERIOD, self.control_loop)

    def odometry_callback(self, msg):
        self.position = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.orientation = units.get_z_rotation_from_quaternion(msg.pose.pose.orientation.x,
                                                                msg.pose.pose.orientation.y, 
                                                                msg.pose.pose.orientation.z, 
                                                                msg.pose.pose.orientation.w)
        
    def goal_callback(self, msg):
        self.goal = [msg.x, msg.y]

    def control_loop(self):
        self.distance_error = np.sqrt((self.goal[0] - self.position[0])**2 + (self.goal[1] - self.position[1])**2)
        self.p2p_orientation = np.arctan2(self.goal[1] - self.position[1], self.goal[0] - self.position[0])
        angle_1  = self.p2p_orientation - self.orientation
        angle_2 = 2*np.pi - abs(self.p2p_orientation - self.orientation)
        self.orientation_error = angle_1 if abs(angle_1) < abs(angle_2) else angle_2

        linear_velocity = 0.0
        angular_velocity = 0.0
        if self.distance_error > self.get_parameter('tolerance_distance').value:
            if abs(self.orientation_error) > self.get_parameter('angle_range').value/2:
                linear_velocity = 0.0
                angular_velocity = self.get_parameter('angular_kp').value * self.orientation_error
            else:
                linear_velocity = self.get_parameter('linear_kp').value * self.distance_error
                angular_velocity = self.get_parameter('angular_kp').value * self.orientation_error

        if (linear_velocity-self.cmd_vel.linear.x) > self.get_parameter('linear_acc').value:
            linear_velocity = self.cmd_vel.linear.x + self.get_parameter('linear_acc').value
        if (angular_velocity-self.cmd_vel.angular.z) > self.get_parameter('angular_acc').value:
            angular_velocity = self.cmd_vel.angular.z + self.get_parameter('angular_acc').value
        
        self.cmd_vel.linear.x = linear_velocity
        self.cmd_vel.angular.z = angular_velocity
        self.cmd_vel_publisher.publish(self.cmd_vel)

def main():
    rclpy.init()
    hard_switch_control = HardSwitchControl()
    rclpy.spin(hard_switch_control)
    hard_switch_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
