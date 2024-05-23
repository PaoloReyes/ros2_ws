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
                ('green_lower', [64, 130, 130]),
                ('green_upper', [99, 190, 190]),
                ('yellow_lower', [20, 15, 212]),
                ('yellow_upper', [50, 40, 245]),
                ('area_threshold', 20),
            ]
        )

        ### cv2 Bridge ###
        self.bridge = CvBridge() 
  
        ### Subscribers ###
        self.create_subscription(Image, '/video_source/raw', self.camera_callback, 10)

        ### Publishers ###
        self.processed_image_publisher = self.create_publisher(Image, '/paolo/processed_img', 10)
        
        self.get_logger().info('traffic_light_identification Node started')
  
    def camera_callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsvFrame = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

        red_lower = np.array(list(self.get_parameter('red_lower').get_parameter_value().integer_array_value), np.uint8)
        red_upper = np.array(list(self.get_parameter('red_upper').get_parameter_value().integer_array_value), np.uint8)

        green_lower = np.array(list(self.get_parameter('green_lower').get_parameter_value().integer_array_value), np.uint8)
        green_upper = np.array(list(self.get_parameter('green_upper').get_parameter_value().integer_array_value), np.uint8)

        yellow_lower = np.array(list(self.get_parameter('yellow_lower').get_parameter_value().integer_array_value), np.uint8)
        yellow_upper = np.array(list(self.get_parameter('yellow_upper').get_parameter_value().integer_array_value), np.uint8)
        
        red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)
        green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)
        yellow_mask = cv2.inRange(hsvFrame, yellow_lower, yellow_upper)

        kernel = np.ones((5, 5), "uint8")

        red_mask = cv2.dilate(red_mask, kernel) 

        green_mask = cv2.dilate(green_mask, kernel)

        yellow_mask = cv2.dilate(yellow_mask, kernel)

        contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
        self.add_contours(cv_img, contours, 'RED')

        contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
        self.add_contours(cv_img, contours, 'GREEN')

        contours, _ = cv2.findContours(yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        self.add_contours(cv_img, contours, 'YELLOW')

        self.processed_image_publisher.publish(self.bridge.cv2_to_imgmsg(cv_img, 'bgr8'))

    def add_contours(self, img, contours, color):
        color_parser = {'RED': (0, 0, 255),
                       'GREEN': (0, 255, 0),
                       'YELLOW': (0, 255, 204),}

        for _, contour in enumerate(contours): 
            area = cv2.contourArea(contour) 
            if(area > self.get_parameter('area_threshold').get_parameter_value().integer_value):
                x, y, w, h = cv2.boundingRect(contour) 
                img = cv2.rectangle(img, (x, y),
                                        (x + w, y + h),
                                        color_parser[color], 2)
  
def main(args=None): 
    rclpy.init(args=args) 
    traffic_light_identifier = TrafficLightIdentifier() 
    rclpy.spin(traffic_light_identifier)
    traffic_light_identifier.destroy_node() 
    rclpy.shutdown()
  
if __name__ == '__main__': 
    main() 