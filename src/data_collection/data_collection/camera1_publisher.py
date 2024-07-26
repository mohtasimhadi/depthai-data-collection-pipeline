import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import depthai as dai

class Camera1Publisher(Node):
    def __init__(self):
        super().__init__('camera1_publisher')
        self.publisher_rgb = self.create_publisher(Image, 'camera1/rgb', 10)
        self.publisher_left = self.create_publisher(Image, 'camera1/left', 10)
        self.publisher_right = self.create_publisher(Image, 'camera1/right', 10)
        self.bridge = CvBridge()

        # Define the pipeline
        self.pipeline = dai.Pipeline()

        # Create color camera
        camRgb = self.pipeline.createColorCamera()
        camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        xoutRgb = self.pipeline.createXLinkOut()
        xoutRgb.setStreamName("rgb")
        camRgb.video.link(xoutRgb.input)

        # Create left and right mono cameras
        monoLeft = self.pipeline.createMonoCamera()
        monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
        
        monoRight = self.pipeline.createMonoCamera()
        monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)

        xoutLeft = self.pipeline.createXLinkOut()
        xoutLeft.setStreamName("left")
        monoLeft.out.link(xoutLeft.input)

        xoutRight = self.pipeline.createXLinkOut()
        xoutRight.setStreamName("right")
        monoRight.out.link(xoutRight.input)

        # Connect to device with specific ID
        self.device = dai.Device(pipeline=self.pipeline)
        self.qRgb = self.device.getOutputQueue(name="rgb", maxSize=30, blocking=True)
        self.qLeft = self.device.getOutputQueue(name="left", maxSize=30, blocking=True)
        self.qRight = self.device.getOutputQueue(name="right", maxSize=30, blocking=True)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        if self.qRgb.has():
            frame = self.qRgb.get().getCvFrame()
            image_message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_rgb.publish(image_message)

        if self.qLeft.has():
            frame = self.qLeft.get().getCvFrame()
            image_message = self.bridge.cv2_to_imgmsg(frame, encoding="mono8")
            self.publisher_left.publish(image_message)

        if self.qRight.has():
            frame = self.qRight.get().getCvFrame()
            image_message = self.bridge.cv2_to_imgmsg(frame, encoding="mono8")
            self.publisher_right.publish(image_message)

def main(args=None):
    rclpy.init(args=args)
    node = Camera1Publisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

