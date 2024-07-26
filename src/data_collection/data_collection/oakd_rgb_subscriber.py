#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class OakDRgbSubscriber(Node):
    def __init__(self):
        super().__init__('oakd_rgb_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'oakd_rgb_image',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Display the image
        cv2.imshow("RGB Camera", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = OakDRgbSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

