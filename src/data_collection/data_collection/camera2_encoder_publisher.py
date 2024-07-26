import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import depthai as dai
import base64

class Camera2EncoderPublisher(Node):
    def __init__(self):
        super().__init__('camera2_encoder_publisher')

        self.publisher_rgb = self.create_publisher(String, 'camera2/rgb_encoded', 10)
        self.publisher_left = self.create_publisher(String, 'camera2/left_encoded', 10)
        self.publisher_right = self.create_publisher(String, 'camera2/right_encoded', 10)

        self.pipeline = dai.Pipeline()

        # Create RGB camera node and encoder
        camRgb = self.pipeline.createColorCamera()
        camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setVideoSize(1920, 1080)
        videoEncRgb = self.pipeline.createVideoEncoder()
        videoEncRgb.setDefaultProfilePreset(1920, 1080, 30, dai.VideoEncoderProperties.Profile.H265_MAIN)
        camRgb.video.link(videoEncRgb.input)
        rgbOut = self.pipeline.createXLinkOut()
        rgbOut.setStreamName("rgb")
        videoEncRgb.bitstream.link(rgbOut.input)

        # Create left and right mono cameras and encoders
        monoLeft = self.pipeline.createMonoCamera()
        monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
        videoEncLeft = self.pipeline.createVideoEncoder()
        videoEncLeft.setDefaultProfilePreset(1280, 720, 30, dai.VideoEncoderProperties.Profile.H264_BASELINE)
        monoLeft.out.link(videoEncLeft.input)
        leftOut = self.pipeline.createXLinkOut()
        leftOut.setStreamName("left")
        videoEncLeft.bitstream.link(leftOut.input)

        monoRight = self.pipeline.createMonoCamera()
        monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
        videoEncRight = self.pipeline.createVideoEncoder()
        videoEncRight.setDefaultProfilePreset(1280, 720, 30, dai.VideoEncoderProperties.Profile.H264_BASELINE)
        monoRight.out.link(videoEncRight.input)
        rightOut = self.pipeline.createXLinkOut()
        rightOut.setStreamName("right")
        videoEncRight.bitstream.link(rightOut.input)

        self.device = dai.Device(self.pipeline)
        self.rgbQueue = self.device.getOutputQueue(name="rgb", maxSize=30, blocking=True)
        self.leftQueue = self.device.getOutputQueue(name="left", maxSize=30, blocking=True)
        self.rightQueue = self.device.getOutputQueue(name="right", maxSize=30, blocking=True)

        self.timer = self.create_timer(1/30, self.publish_frames)

    def publish_frames(self):
        if self.rgbQueue.has():
            rgb_data = self.rgbQueue.get().getData()
            self.publisher_rgb.publish(String(data=base64.b64encode(rgb_data).decode('utf-8')))

        if self.leftQueue.has():
            left_data = self.leftQueue.get().getData()
            self.publisher_left.publish(String(data=base64.b64encode(left_data).decode('utf-8')))

        if self.rightQueue.has():
            right_data = self.rightQueue.get().getData()
            self.publisher_right.publish(String(data=base64.b64encode(right_data).decode('utf-8')))

def main(args=None):
    rclpy.init(args=args)
    node = Camera2EncoderPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

