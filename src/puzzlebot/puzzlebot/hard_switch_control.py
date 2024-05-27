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
                ('tolerance_distance', rclpy.parameter.Parameter.Type.DOUBLE),
                ('linear_kp', rclpy.parameter.Parameter.Type.DOUBLE),
                ('angular_kp', rclpy.parameter.Parameter.Type.DOUBLE),
                ('linear_acc', rclpy.parameter.Parameter.Type.DOUBLE),
                ('angular_acc', rclpy.parameter.Parameter.Type.DOUBLE),
                ('avoiding_d', rclpy.parameter.Parameter.Type.DOUBLE),
                ('stop_d', rclpy.parameter.Parameter.Type.DOUBLE),
                ('lidar_vel', rclpy.parameter.Parameter.Type.DOUBLE),
                ('lidar_turning_kp', rclpy.parameter.Parameter.Type.DOUBLE),
                ('epsilon', rclpy.parameter.Parameter.Type.DOUBLE),
            ]
        )

        ## Variables ##
        self.position = [0, 0]
        self.orientation = 0
        self.goal = [0, 0]
        self.cmd_vel = Twist()
        self.lidar_vel = Twist()
        self.lidar_action = False

        ## Subscribers ##
        self.create_subscription(Odometry, '/odometry', self.odometry_callback, 10)
        self.create_subscription(Pose2D, '/goal', self.goal_callback, 10)
        self.create_subscription(LaserScan, '/lidar', self.lidar_callback, 10)

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
        distance_error = np.sqrt((self.goal[0] - self.position[0])**2 + (self.goal[1] - self.position[1])**2)
        p2p_orientation = np.arctan2(self.goal[1] - self.position[1], self.goal[0] - self.position[0])
        angle_1  = p2p_orientation - self.orientation
        angle_2 = 2*np.pi - abs(p2p_orientation - self.orientation)
        orientation_error = angle_1 if abs(angle_1) < abs(angle_2) else angle_2

        linear_velocity = 0.0
        angular_velocity = 0.0
        at_goal = distance_error < self.get_parameter('tolerance_distance').get_parameter_value().double_value
        if not at_goal:
            linear_velocity = self.get_parameter('linear_kp').get_parameter_value().double_value * distance_error
            angular_velocity = self.get_parameter('angular_kp').get_parameter_value().double_value * orientation_error

        if (linear_velocity-self.cmd_vel.linear.x) > self.get_parameter('linear_acc').get_parameter_value().double_value:
            linear_velocity = self.cmd_vel.linear.x + self.get_parameter('linear_acc').get_parameter_value().double_value
        if (angular_velocity-self.cmd_vel.angular.z) > self.get_parameter('angular_acc').get_parameter_value().double_value:
            angular_velocity = self.cmd_vel.angular.z + self.get_parameter('angular_acc').get_parameter_value().double_value

        if self.lidar_action and not at_goal:
            self.cmd_vel.linear.x = self.lidar_vel.linear.x
            self.cmd_vel.angular.z = self.lidar_vel.angular.z
        else:
            self.cmd_vel.linear.x = linear_velocity
            self.cmd_vel.angular.z = angular_velocity
        
        self.cmd_vel_publisher.publish(self.cmd_vel)

    def lidar_callback(self, msg):
        ranges = msg.ranges
        valid_point = False
        for range in ranges: 
            if range != float('inf'): 
                valid_point = True
                break
        if valid_point:
            closest_point = min(ranges)
            closest_index = ranges.index(closest_point)
            angle_of_point = msg.angle_min+(closest_index*msg.angle_increment)
            distance_error = closest_point
            if abs(angle_of_point) < np.pi/2:
                angle_error = angle_of_point+np.pi
                angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))
                if distance_error < self.get_parameter('avoiding_d').get_parameter_value().double_value:
                    self.lidar_action = True
                    if distance_error < self.get_parameter('stop_d').get_parameter_value().double_value:
                        self.lidar_vel.linear.x = 0.0
                        self.lidar_vel.angular.z = 0.0
                    else:
                        self.lidar_vel.linear.x = self.get_parameter('lidar_vel').get_parameter_value().double_value
                        self.lidar_vel.angular.z = self.get_parameter('lidar_turning_kp').get_parameter_value().double_value*angle_error
                else:
                    self.lidar_action = False
            else:
                self.lidar_action = False

            if self.lidar_action and distance_error > self.get_parameter('avoiding_d').get_parameter_value().double_value + self.get_parameter('epsilon').get_parameter_value().double_value:
                self.lidar_action = False
        else:
            self.lidar_action = False

def main():
    rclpy.init()
    hard_switch_control = HardSwitchControl()
    rclpy.spin(hard_switch_control)
    hard_switch_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
