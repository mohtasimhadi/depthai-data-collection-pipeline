# gps_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading

class GPSPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')
        self.publisher_ = self.create_publisher(String, 'gnrmc_data', 10)
        self.serial_port = '/dev/ttyACM0'
        self.serial_baudrate = 115200  # Use the correct baud rate for your GPS device
        self.serial_conn = serial.Serial(self.serial_port, self.serial_baudrate, timeout=1)
        self.read_thread = threading.Thread(target=self.read_serial)
        self.read_thread.daemon = True
        self.read_thread.start()

    def read_serial(self):
        while rclpy.ok():
            line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('$GNRMC'):
                self.publisher_.publish(String(data=line))

def main(args=None):
    rclpy.init(args=args)
    node = GPSPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

