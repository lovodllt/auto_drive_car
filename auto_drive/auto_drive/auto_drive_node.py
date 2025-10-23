import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import os

class AutoDriveNode(Node):
    def __init__(self):
        super().__init__('auto_drive_node')
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel_unstamped', 1)

        self.timer = self.create_timer(0.02, self.timer_callback)

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/oakd/rgb/preview/image_raw', self.image_callback, 1)

        self.linear_velocity = 1.2
        self.yaw_speed = 0.0

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([15, 50, 50])
        upper_yellow = np.array([45, 255, 255])

        mask= cv2.inRange(hsv_image, lower_yellow, upper_yellow)

        cv2.imshow('img', cv_image)
        cv2.waitKey(1)

        mask = mask[220:, :]
        moments = cv2.moments(mask)

        try:
            xc = int(moments['m10'] / moments['m00'])
            self.yaw_speed = (160 - xc) * 0.08
        except ZeroDivisionError:
            self.get_logger().error("No yellow line detected.")
            self.yaw_speed = 0.0
            self.linear_velocity
            pass

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.linear_velocity
        msg.angular.z = self.yaw_speed
        self.vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    auto_drive_node = AutoDriveNode()

    rclpy.spin(auto_drive_node)

    auto_drive_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()    
        