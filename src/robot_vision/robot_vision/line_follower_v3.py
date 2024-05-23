import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower_node')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, 'video_source/raw', self.camera_callback, 10)
        self.pub = self.create_publisher(Image, 'processed_img', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.robot_vel = Twist()
        self.image_received_flag = False
        dt = 0.1
        self.timer = self.create_timer(dt, self.timer_callback)
        self.get_logger().info('Line Follower Node started')

    def camera_callback(self, msg):
        try:
            self.cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_received_flag = True
        except Exception as e:
            self.get_logger().info('Failed to get an image: {}'.format(str(e)))

    def timer_callback(self):
        if self.image_received_flag:
            image = self.cv_img.copy()
            image = cv2.flip(image, 0)
            image = cv2.resize(image, (240, 320))
            # region of interest, being the lowest part of the camera
            region_of_interest = image[161:240, 0:320]  
            #blurred = cv2.GaussianBlur(image, (5, 5), 0) # gaussian filter
            gray = cv2.cvtColor(region_of_interest, cv2.COLOR_BGR2GRAY)
            _, thresholded = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
            # Adaptative threshold
            #thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2) 
        
            contours, _ = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                # Assume the largest contour is the line
                largest_contour = max(contours, key=cv2.contourArea)
                M = cv2.moments(largest_contour)
                if M['m00'] != 0:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    # Draw a circle at the center of the line
                    mark_size = 10  # Puedes ajustar esto según la estética deseada

                    # Dibujar un círculo en el centro de la línea
                    cv2.circle(image, (cx, cy), 10, (0, 255, 0), -1)

                    # Calculate error from the center of the image
                    error = cx - image.shape[1] // 2
                    self.robot_vel.angular.z = -float(error) / 100  # P-controller for steering
                    self.robot_vel.linear.x = 0.2  # Constant forward speed
                else:
                    self.robot_vel.angular.z = 0.0
                    self.robot_vel.linear.x = 0.0
            else:
                self.get_logger().info('No line detected')
                self.robot_vel.angular.z = 0.0
                self.robot_vel.linear.x = 0.0

            self.cmd_vel_pub.publish(self.robot_vel)
            self.pub.publish(self.bridge.cv2_to_imgmsg(image, 'bgr8'))

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()