import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import depthai as dai
import cv2

class OakDMultiCameraPublisher(Node):
    def __init__(self):
        super().__init__('oakd_multi_camera_publisher')
        self.rgb_publisher = self.create_publisher(Image, 'oakd_rgb_image', 10)
        self.left_publisher = self.create_publisher(Image, 'oakd_left_image', 10)
        self.right_publisher = self.create_publisher(Image, 'oakd_right_image', 10)
        self.bridge = CvBridge()

        self.pipeline = self.create_pipeline()
        self.device = dai.Device(self.pipeline)
        self.qRgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        self.qMonoLeft = self.device.getOutputQueue(name="left", maxSize=4, blocking=False)
        self.qMonoRight = self.device.getOutputQueue(name="right", maxSize=4, blocking=False)

        self.timer = self.create_timer(0.1, self.publish_frames)

    def create_pipeline(self):
        pipeline = dai.Pipeline()

        camRgb = pipeline.createColorCamera()
        camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        monoLeft = pipeline.createMonoCamera()
        monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

        monoRight = pipeline.createMonoCamera()
        monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

        xoutRgb = pipeline.createXLinkOut()
        xoutRgb.setStreamName("rgb")

        xoutMonoLeft = pipeline.createXLinkOut()
        xoutMonoLeft.setStreamName("left")

        xoutMonoRight = pipeline.createXLinkOut()
        xoutMonoRight.setStreamName("right")

        camRgb.video.link(xoutRgb.input)
        monoLeft.out.link(xoutMonoLeft.input)
        monoRight.out.link(xoutMonoRight.input)

        return pipeline

    def publish_frames(self):
        inRgb = self.qRgb.tryGet()
        inMonoLeft = self.qMonoLeft.tryGet()
        inMonoRight = self.qMonoRight.tryGet()

        if inRgb is not None:
            frameRgb = inRgb.getCvFrame()
            msg = self.bridge.cv2_to_imgmsg(frameRgb, "bgr8")
            self.rgb_publisher.publish(msg)

        if inMonoLeft is not None:
            frameMonoLeft = inMonoLeft.getCvFrame()
            msg = self.bridge.cv2_to_imgmsg(frameMonoLeft, "mono8")
            self.left_publisher.publish(msg)

        if inMonoRight is not None:
            frameMonoRight = inMonoRight.getCvFrame()
            msg = self.bridge.cv2_to_imgmsg(frameMonoRight, "mono8")
            self.right_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OakDMultiCameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

