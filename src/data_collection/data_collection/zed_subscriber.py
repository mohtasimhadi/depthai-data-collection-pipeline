# zed_subscriber.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import numpy as np
import base64
from datetime import datetime

class ZEDSubscriber(Node):
    def __init__(self):
        super().__init__('zed_subscriber')

        self.subscription_left = self.create_subscription(
            String,
            'zed/left_camera',
            self.listener_callback_left,
            10)
        
        self.subscription_right = self.create_subscription(
            String,
            'zed/right_camera',
            self.listener_callback_right,
            10)

        self.frames_left = []
        self.frames_right = []

    def listener_callback_left(self, msg):
        frame_data = base64.b64decode(msg.data)
        frame = np.frombuffer(frame_data, dtype=np.uint8).reshape((720, 1280, 4))  # Update shape as per ZED camera resolution
        self.frames_left.append(frame)

    def listener_callback_right(self, msg):
        frame_data = base64.b64decode(msg.data)
        frame = np.frombuffer(frame_data, dtype=np.uint8).reshape((720, 1280, 4))  # Update shape as per ZED camera resolution
        self.frames_right.append(frame)

    def save_videos(self):
        # Save left camera video
        left_out = cv2.VideoWriter(
            f'/home/bizon/Documents/faraz_files/Zed data/left_camera/left_{datetime.now().strftime("%Y%m%d_%H%M%S")}.avi',
            cv2.VideoWriter_fourcc(*'XVID'),
            30,
            (1280, 720))  # Update resolution as per ZED camera
        
        for frame in self.frames_left:
            left_out.write(frame)
        left_out.release()

        # Save right camera video
        right_out = cv2.VideoWriter(
            f'/home/bizon/Documents/faraz_files/Zed data/right_camera/right_{datetime.now().strftime("%Y%m%d_%H%M%S")}.avi',
            cv2.VideoWriter_fourcc(*'XVID'),
            30,
            (1280, 720))  # Update resolution as per ZED camera
        
        for frame in self.frames_right:
            right_out.write(frame)
        right_out.release()

def main(args=None):
    rclpy.init(args=args)
    node = ZEDSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_videos()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

