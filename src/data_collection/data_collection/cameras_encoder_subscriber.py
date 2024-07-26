import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import base64
import os

class CameraEncoderSubscriber(Node):
    def __init__(self):
        super().__init__('cameras_encoder_subscriber')

        # Create subscriptions for camera1
        self.create_subscription(String, 'camera1/rgb_encoded', self.camera1_rgb_callback, 10)
        self.create_subscription(String, 'camera1/left_encoded', self.camera1_left_callback, 10)
        self.create_subscription(String, 'camera1/right_encoded', self.camera1_right_callback, 10)

        # Create subscriptions for camera2
        self.create_subscription(String, 'camera2/rgb_encoded', self.camera2_rgb_callback, 10)
        self.create_subscription(String, 'camera2/left_encoded', self.camera2_left_callback, 10)
        self.create_subscription(String, 'camera2/right_encoded', self.camera2_right_callback, 10)

        # Define file paths
        self.paths = {
            "camera1_rgb": "/home/bsen101/Documents/faraz_files/Videos/camera1_rgb/output.h265",
            "camera1_left": "/home/bsen101/Documents/faraz_files/Videos/camera1_left/output.h264",
            "camera1_right": "/home/bsen101/Documents/faraz_files/Videos/camera1_right/output.h264",
            "camera2_rgb": "/home/bsen101/Documents/faraz_files/Videos/camera2_rgb/output.h265",
            "camera2_left": "/home/bsen101/Documents/faraz_files/Videos/camera2_left/output.h264",
            "camera2_right": "/home/bsen101/Documents/faraz_files/Videos/camera2_right/output.h264",
        }

        # Clear existing files
        for path in self.paths.values():
            if os.path.exists(path):
                os.remove(path)

    def save_encoded_data(self, data, path):
        with open(path, "ab") as f:
            f.write(base64.b64decode(data))

    def camera1_rgb_callback(self, msg):
        self.save_encoded_data(msg.data, self.paths["camera1_rgb"])

    def camera1_left_callback(self, msg):
        self.save_encoded_data(msg.data, self.paths["camera1_left"])

    def camera1_right_callback(self, msg):
        self.save_encoded_data(msg.data, self.paths["camera1_right"])

    def camera2_rgb_callback(self, msg):
        self.save_encoded_data(msg.data, self.paths["camera2_rgb"])

    def camera2_left_callback(self, msg):
        self.save_encoded_data(msg.data, self.paths["camera2_left"])

    def camera2_right_callback(self, msg):
        self.save_encoded_data(msg.data, self.paths["camera2_right"])

def main(args=None):
    rclpy.init(args=args)
    node = CameraEncoderSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

