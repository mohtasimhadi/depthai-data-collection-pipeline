import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class MultiCameraSubscriber(Node):
    def __init__(self):
        super().__init__('multi_camera_subscriber')

        self.bridge = CvBridge()

        # Subscriptions for camera 1
        self.subscription_rgb1 = self.create_subscription(
            Image,
            'camera1/rgb',
            self.rgb1_callback,
            10)
        self.subscription_left1 = self.create_subscription(
            Image,
            'camera1/left',
            self.left1_callback,
            10)
        self.subscription_right1 = self.create_subscription(
            Image,
            'camera1/right',
            self.right1_callback,
            10)

        # Subscriptions for camera 2
        self.subscription_rgb2 = self.create_subscription(
            Image,
            'camera2/rgb',
            self.rgb2_callback,
            10)
        self.subscription_left2 = self.create_subscription(
            Image,
            'camera2/left',
            self.left2_callback,
            10)
        self.subscription_right2 = self.create_subscription(
            Image,
            'camera2/right',
            self.right2_callback,
            10)

        # Initialize video writers
        self.init_video_writers()

    def init_video_writers(self):
        self.video_writers = {}
        folders = {
            'camera1_left': '/home/bsen101/Documents/faraz_files/Videos/camera1_left',
            'camera1_right': '/home/bsen101/Documents/faraz_files/Videos/camera1_right',
            'camera2_left': '/home/bsen101/Documents/faraz_files/Videos/camera2_left',
            'camera2_right': '/home/bsen101/Documents/faraz_files/Videos/camera2_right',
            'camera1_rgb': '/home/bsen101/Documents/faraz_files/Videos/camera1_rgb',
            'camera2_rgb': '/home/bsen101/Documents/faraz_files/Videos/camera2_rgb'
        }

        for key, folder in folders.items():
            if not os.path.exists(folder):
                os.makedirs(folder)
            for f in os.listdir(folder):
                os.remove(os.path.join(folder, f))

            self.video_writers[key] = cv2.VideoWriter(
                os.path.join(folder, 'output.avi'),
                cv2.VideoWriter_fourcc(*'DIVX'),
                30,
                (1920, 1080) if 'rgb' in key else (1280, 720),
                isColor=True if 'rgb' in key else False
            )

    def rgb1_callback(self, msg):
        self.display_and_save_image(msg, "RGB Camera 1", "camera1_rgb")

    def left1_callback(self, msg):
        self.display_and_save_image(msg, "Left Mono Camera 1", "camera1_left")

    def right1_callback(self, msg):
        self.display_and_save_image(msg, "Right Mono Camera 1", "camera1_right")

    def rgb2_callback(self, msg):
        self.display_and_save_image(msg, "RGB Camera 2", "camera2_rgb")

    def left2_callback(self, msg):
        self.display_and_save_image(msg, "Left Mono Camera 2", "camera2_left")

    def right2_callback(self, msg):
        self.display_and_save_image(msg, "Right Mono Camera 2", "camera2_right")

    def display_and_save_image(self, msg, window_name, writer_key=None):
        encoding = 'bgr8' if 'rgb' in window_name.lower() else 'mono8'
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding=encoding)
        cv2.imshow(window_name, cv_image)

        if writer_key:
            self.video_writers[writer_key].write(cv_image)
        
        cv2.waitKey(1)

    def destroy_node(self):
        for writer in self.video_writers.values():
            writer.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

