import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class OakDMultiCameraSubscriber(Node):
    def __init__(self):
        super().__init__('oakd_multi_camera_subscriber')
        self.rgb_subscription = self.create_subscription(
            Image,
            'oakd_rgb_image',
            self.rgb_callback,
            10)
        self.left_subscription = self.create_subscription(
            Image,
            'oakd_left_image',
            self.left_callback,
            10)
        self.right_subscription = self.create_subscription(
            Image,
            'oakd_right_image',
            self.right_callback,
            10)

        self.bridge = CvBridge()
        self.rgb_image = None
        self.left_image = None
        self.right_image = None

    def rgb_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.display_images()

    def left_callback(self, msg):
        self.left_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
        self.display_images()

    def right_callback(self, msg):
        self.right_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
        self.display_images()

    def display_images(self):
        if self.rgb_image is not None and self.left_image is not None and self.right_image is not None:
            # Display the RGB image
            cv2.imshow("RGB Camera", self.rgb_image)

            # Display the left mono image
            cv2.imshow("Left Mono Camera", self.left_image)

            # Display the right mono image
            cv2.imshow("Right Mono Camera", self.right_image)

            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = OakDMultiCameraSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

