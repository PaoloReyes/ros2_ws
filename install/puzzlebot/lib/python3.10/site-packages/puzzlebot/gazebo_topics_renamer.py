import rclpy
import numpy as np
import sys

from rclpy.node import Node

from .submodules import topic_utils as tu

class GazeboTopicRenamer(Node):
	def __init__(self):
		super().__init__('gazebo_topics_renamer')

		self.declare_parameter('topics_map', rclpy.Parameter.Type.STRING_ARRAY)
		try:
			self.topics_to_map = self.get_parameter('topics_map').get_parameter_value().string_array_value
		except:
			self.get_logger().error('No topics map found in parameters')
			sys.exit(1)

		for topic in self.topics_to_map:
			topic_data = topic.split(';')
			self.create_subscription(tu.topic_types[topic_data[0]], topic_data[1], self.resend_msg, 10)
			#self.create_publisher(tu.topic_types[topic_data[0]], topic_data[2], 10)

	def resend_msg(self, msg):
		self.get_logger().info(msg)

		# #### Subscribers ####
		# self.create_subscription(TFMessage, '/model/Puzzlebot/tf', self.resend_tf, 10)
		# self.create_subscription(Odometry, '/model/Puzzlebot/odometry', self.wheel_transforms, 10)
		# self.create_subscription(LaserScan, '/raw_lidar', self.lidar_transform, 10)

		# #### Publisher ####
		# self.tf = self.create_publisher(TFMessage, '/tf', 10)
		# self.lidar_pub = self.create_publisher(LaserScan, '/lidar', 10)


	# def lidar_transform(self, msg):
	# 	msg.header.frame_id = 'Lidar_link'
	# 	self.lidar_pub.publish(msg)

def main(args=None): 
	rclpy.init(args=args) 
	f_p = GazeboTopicRenamer() 
	rclpy.spin(f_p) 
	f_p.destroy_node() 
	rclpy.shutdown() 

if __name__ == '__main__': 
	main() 