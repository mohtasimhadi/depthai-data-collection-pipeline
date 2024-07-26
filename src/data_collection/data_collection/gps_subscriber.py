
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import csv
from datetime import datetime

class CSVSubscriber(Node):
    def __init__(self):
        super().__init__('csv_subscriber')
        self.subscription = self.create_subscription(String, 'gnrmc_data', self.listener_callback, 10)
        self.csv_filename = 'gnrmc_data.csv'
        self.initialize_csv()

    def initialize_csv(self):
        with open(self.csv_filename, mode='w', newline='') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(['Timestamp', 'GNRMC Data'])

    def listener_callback(self, msg):
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        gnrmc_data = msg.data
        with open(self.csv_filename, mode='a', newline='') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow([timestamp, gnrmc_data])

def main(args=None):
    rclpy.init(args=args)
    node = CSVSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

