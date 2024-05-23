import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from .submodules.functions import *
from std_msgs.msg import Int8, Bool
from geometry_msgs.msg import Twist
import time

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(Image,'video_source/raw',self.camera_callback,10)

        self.pub = self.create_publisher(Image, 'line_image', 10)
        #self.pub2 = self.create_publisher(Image, 'wraped', 10)
        #self.pub3 = self.create_publisher(Image, 'original_frame', 10)
        
        self.control_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.control_msg = Twist()
        self.i = 0

        self.recieved_flag = False

        self.ready_for_operation = False

        # Prooportional gain for the steering error
        self.Kc = 0.025

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('Line Follower Node has been started!!!')

    def camera_callback(self, msg):
        try:
            self.msg_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.recieved_flag = True
        except:
            self.get_logger().info('Error in converting image')

    def timer_callback(self):
        if self.recieved_flag:

            frame = self.msg_image.copy()

            frame = cv2.resize(frame, (240, 240))
            frame = cv2.flip(frame, 0)
            frame = cv2.flip(frame, 1)

            center = np.array([0, 0])

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            lower_white = np.array([0, 0, 0], np.uint8)
            upper_white = np.array([180, 100, 100], np.uint8)

            mask = cv2.inRange(hsv, lower_white, upper_white)

            #mask = cv2.bitwise_not(mask)

            center_image = mask[220:240, 50:190]


            mid_x = frame.shape[0] // 2
            mid_y = center_image.shape[1] // 2

            center_x = 0
            center_y = 0

            contours, _ = cv2.findContours(center_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                c = max(contours, key=cv2.contourArea)
                M = cv2.moments(c)
                ((x_r, y_r), radius) = cv2.minEnclosingCircle(c)
            
                if M["m00"] != 0:
                    center[0] = int(M['m10']/M['m00'])
                    center[1] = int(M['m01']/M['m00'])
                    cv2.drawContours(frame, c, -1, (0,255,0), 1)

                    center[0] = center[0] + 50

                    error = center[0] - frame.shape[1] // 2
                    self.control_msg.linear.x = 0.2
                    self.control_msg.angular.z = -self.Kc * error
                else:
                    self.control_msg.linear.x = 0.0
                    self.control_msg.angular.z = 0.0                    

            else:
                #print("No contours found")
                self.control_msg.linear.x = 0.0
                self.control_msg.angular.z = 0.0

            error = center_error(center_x, center_y)
            print(error)

            cv2.line(frame, (mid_x, mid_y - 10), (mid_x, mid_y + 10), (0, 0, 255), 1)
            cv2.line(frame, (mid_x - 10, mid_y), (mid_x + 10, mid_y), (0, 0, 255), 1)
            cv2.circle(frame, (int(center[0]), int(center[1])), 5, (0, 255, 0), -1)

            print(self.control_msg.angular.z)
            #result = cv2.addWeighted(frame, 1, center_image, 0.5, 0)

            self.control_pub.publish(self.control_msg)
            self.pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))
            self.recieved_flag = False
    

def main():
    rclpy.init()
    line_follower = LineFollower()
    rclpy.spin(line_follower)
    line_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()