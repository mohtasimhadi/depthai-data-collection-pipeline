#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import depthai as dai
import cv2

class OakDRgbPublisher(Node):
    def __init__(self):
        super().__init__('oakd_rgb_publisher')
        self.publisher_ = self.create_publisher(Image, 'oakd_rgb_image', 10)
        self.bridge = CvBridge()
        self.pipeline = self.create_pipeline()

        self.device = dai.Device(self.pipeline)
        self.qRgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

        self.timer = self.create_timer(0.1, self.publish_frame)

    def create_pipeline(self):
        pipeline = dai.Pipeline()
        camRgb = pipeline.createColorCamera()
        camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        xoutRgb = pipeline.createXLinkOut()
        xoutRgb.setStreamName("rgb")
        camRgb.video.link(xoutRgb.input)

        return pipeline

    def publish_frame(self):
        inRgb = self.qRgb.get()
        frameRgb = inRgb.getCvFrame()
        msg = self.bridge.cv2_to_imgmsg(frameRgb, "bgr8")
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OakDRgbPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

