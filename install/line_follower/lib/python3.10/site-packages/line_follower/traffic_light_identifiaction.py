import rclpy 

from rclpy.node import Node 

import cv2 
import numpy as np 

from cv_bridge import CvBridge 

from sensor_msgs.msg import Image 

class TrafficLightIdentifier(Node): 
    def __init__(self): 
        super().__init__('traffic_light_identification') 

        self.declare_parameters(
            namespace='',
            parameters=[
                ('red_lower', [152, 31, 205]),
                ('red_upper', [192, 91, 255]),
                ('green_lower', [136, 87, 111]),
                ('green_upper', [136, 87, 111]),
                ('yellow_lower', [136, 87, 111]),
                ('yellow_upper', [136, 87, 111]),
            ]
        )


        ### cv2 Bridge ###
        self.bridge = CvBridge() 
  
        ### Subscribers ###
        self.raw_camera_subscriber = self.create_subscription(Image, '/video_source/raw', self.camera_callback, 10)

        ### Publishers ###
        self.processed_image_publisher = self.create_publisher(Image, '/paolo/processed_img', 10) 
        
        self.get_logger().info('traffic_light_identification Node started') 
  
    def camera_callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsvFrame = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

        red_lower = self.get_parameter('red_lower').value
        red_upper = self.get_parameter('red_upper').value
        self.get_logger().info(red_lower)
        self.get_logger().info(red_upper)
        red_mask = cv2.inRange(hsvFrame, red_lower, red_upper) 

        self.pub.publish(self.bridge.cv2_to_imgmsg(self.cv_img, 'bgr8'))
  
def main(args=None): 
    rclpy.init(args=args) 
    traffic_light_identifier = TrafficLightIdentifier() 
    rclpy.spin(traffic_light_identifier)
    traffic_light_identifier.destroy_node() 
    rclpy.shutdown() 
  
if __name__ == '__main__': 
    main() 