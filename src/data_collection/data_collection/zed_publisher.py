import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import String
from cv_bridge import CvBridge
import pyzed.sl as sl
import cv2
import numpy as np

class ZedPublisher(Node):
    def __init__(self):
        super().__init__('zed_publisher')
        
        self.publisher_left = self.create_publisher(Image, 'zed/left_image', 10)
        self.publisher_right = self.create_publisher(Image, 'zed/right_image', 10)
        self.publisher_depth = self.create_publisher(Image, 'zed/depth_image', 10)
        self.publisher_imu = self.create_publisher(Imu, 'zed/imu', 10)
        self.publisher_sensor = self.create_publisher(String, 'zed/sensor_data', 10)
        
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD1080
        init_params.camera_fps = 30
        init_params.coordinate_units = sl.UNIT.METER
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f"Failed to open ZED camera: {repr(err)}")
            rclpy.shutdown()

    def timer_callback(self):
        runtime_params = sl.RuntimeParameters()
        if self.zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
            left_image = sl.Mat()
            right_image = sl.Mat()
            depth_image = sl.Mat()

            self.zed.retrieve_image(left_image, sl.VIEW.LEFT)
            self.zed.retrieve_image(right_image, sl.VIEW.RIGHT)
            self.zed.retrieve_measure(depth_image, sl.MEASURE.DEPTH)

            left_frame = left_image.get_data()
            right_frame = right_image.get_data()
            depth_frame = depth_image.get_data()

            left_msg = self.bridge.cv2_to_imgmsg(left_frame, "bgr8")
            right_msg = self.bridge.cv2_to_imgmsg(right_frame, "bgr8")
            depth_msg = self.bridge.cv2_to_imgmsg(depth_frame, "32FC1")

            self.publisher_left.publish(left_msg)
            self.publisher_right.publish(right_msg)
            self.publisher_depth.publish(depth_msg)

            sensors_data = sl.SensorsData()
            if self.zed.get_sensors_data(sensors_data, sl.TIME_REFERENCE.CURRENT) == sl.ERROR_CODE.SUCCESS:
                imu_msg = Imu()
                imu_data = sensors_data.get_imu_data()
                imu_msg.angular_velocity.x = imu_data.get_angular_velocity()[0]
                imu_msg.angular_velocity.y = imu_data.get_angular_velocity()[1]
                imu_msg.angular_velocity.z = imu_data.get_angular_velocity()[2]
                imu_msg.linear_acceleration.x = imu_data.get_linear_acceleration()[0]
                imu_msg.linear_acceleration.y = imu_data.get_linear_acceleration()[1]
                imu_msg.linear_acceleration.z = imu_data.get_linear_acceleration()[2]
                imu_msg.orientation.x = imu_data.get_orientation().get()[0]
                imu_msg.orientation.y = imu_data.get_orientation().get()[1]
                imu_msg.orientation.z = imu_data.get_orientation().get()[2]
                imu_msg.orientation.w = imu_data.get_orientation().get()[3]

                sensor_msg = String()
                sensor_msg.data = (
                    f"IMU Orientation: {imu_msg.orientation.x}, {imu_msg.orientation.y}, "
                    f"{imu_msg.orientation.z}, {imu_msg.orientation.w}\n"
                    f"IMU Acceleration: {imu_msg.linear_acceleration.x}, {imu_msg.linear_acceleration.y}, "
                    f"{imu_msg.linear_acceleration.z}\n"
                    f"IMU Angular Velocity: {imu_msg.angular_velocity.x}, {imu_msg.angular_velocity.y}, "
                    f"{imu_msg.angular_velocity.z}\n"
                )

                self.publisher_imu.publish(imu_msg)
                self.publisher_sensor.publish(sensor_msg)

def main(args=None):
    rclpy.init(args=args)
    zed_publisher = ZedPublisher()
    rclpy.spin(zed_publisher)
    zed_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

