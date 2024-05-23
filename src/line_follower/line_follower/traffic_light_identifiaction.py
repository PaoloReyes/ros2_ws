import rclpy 

from rclpy.node import Node 

import cv2 
import numpy as np 

from cv_bridge import CvBridge 

from sensor_msgs.msg import Image 

class CVExample(Node): 
    def __init__(self): 
        super().__init__('traffic_light_identification') 
 
        self.bridge = CvBridge() 
  
        self.sub = self.create_subscription(Image, 'videosource/raw', self.camera_callback, 10) 
        self.pub = self.create_publisher(Image, 'processed_img', 10) 
        
        self.get_logger().info('traffic_light_identification Node started') 
  
    def camera_callback(self, msg): 
        self.cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")  
        self.pub.publish(self.bridge.cv2_to_imgmsg(self.cv_img, 'bgr8'))
  
def main(args=None): 
    rclpy.init(args=args) 
    cv_e = CVExample() 
    rclpy.spin(cv_e) 
    cv_e.destroy_node() 
    rclpy.shutdown() 
  
if __name__ == '__main__': 
    main() 