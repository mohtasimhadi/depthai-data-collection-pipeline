import cv2
import os
import time
import argparse
import threading
import depthai as dai
from camera_setup.pipeline import get_pipeline
from utils.host_sync import HostSync
from utils.general_helper import save_calibration, get_queues, get_files, write_imu

def main(thread, output_dir):
    pipeline = get_pipeline()
    os.makedirs(output_dir+f'/{thread}_depth')
    with dai.Device(pipeline) as device:
        queues, imu_queue = get_queues()
        sync = HostSync()
        save_calibration(device.readCalibration(), output_dir, thread)
        file_color, file_monoL, file_monoR, file_imus = get_files(output_dir, thread)
        print(f'{thread} started...')
        try:
            while True:
                write_imu(imu_queue.get(), file_imus)
                for queue in queues:
                    new_message = queue.get()
                    message = sync.add_msg(queue.getName(), new_message)
                    if message:
                        file_color.write(message['color'].getData())
                        file_monoL.write(message['monoL'].getData())
                        file_monoR.write(message['monoR'].getData())
                        depth_file_name = f'{thread}_depth/' +str(message['depth'].getSequenceNum()) + f"_{str(message['depth'].getTimestampDevice())}.png"
                        cv2.imwrite(os.path.join(output_dir, depth_file_name), message['depth'].getFrame())
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="DepthAI Data Collection Pipeline")
    parser.add_argument('--output_dir', type=str, required=True, help='Directory where the data will be recorded.')
    args = parser.parse_args()

    output_dir = args.output_dir
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    thread_01 = threading.Thread(target=main, args=('thread_01', output_dir))
    thread_02 = threading.Thread(target=main, args=('thread_02', output_dir))
    thread_01.start()
    time.sleep(2)
    thread_02.start()
    
    thread_01.join()
    thread_02.join()

    print("All recordings stopped.")