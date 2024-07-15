import cv2
import os, json
import time
import argparse
import threading
import depthai as dai
from camera_setup.pipeline import get_pipeline
from utils.host_sync import HostSync

def main(thread, output_dir):
    pipeline = get_pipeline()
    os.makedirs(output_dir+f'/{thread}_depth')
    with dai.Device(pipeline) as device:
        queues = []
        queues.append(device.getOutputQueue(name="depth", maxSize=30, blocking=True))
        queues.append(device.getOutputQueue(name="color", maxSize=30, blocking=True))
        queues.append(device.getOutputQueue(name="monoL", maxSize=30, blocking=True))
        queues.append(device.getOutputQueue(name="monoR", maxSize=30, blocking=True))
        imu_queue = device.getOutputQueue(name='imu', maxSize=30, blocking=True)
        sync = HostSync()

        calibData = device.readCalibration()
        
        calib_dict = {
            'left_intrinsics'           : calibData.getCameraIntrinsics(dai.CameraBoardSocket.CAM_B, dai.Size2f(1080, 720)),
            'right_intrinsics'          : calibData.getCameraIntrinsics(dai.CameraBoardSocket.CAM_C, dai.Size2f(1080, 720)),
            'rgb_intrinsics'            : calibData.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, dai.Size2f(1920, 1080)),
            'left_distortion'           : calibData.getDistortionCoefficients(dai.CameraBoardSocket.CAM_B),
            'right_distortion'          : calibData.getDistortionCoefficients(dai.CameraBoardSocket.CAM_C),
            'rgb_distortion'            : calibData.getDistortionCoefficients(dai.CameraBoardSocket.CAM_A),
            'extrinsics_left_to_right'  : calibData.getCameraExtrinsics(dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C),
            'extrinsics_right_to_left'  : calibData.getCameraExtrinsics(dai.CameraBoardSocket.CAM_C, dai.CameraBoardSocket.CAM_B),
            'extrinsics_left_to_rgb'    : calibData.getCameraExtrinsics(dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_A),
            'extrinsics_right_to_rgb'   : calibData.getCameraExtrinsics(dai.CameraBoardSocket.CAM_C, dai.CameraBoardSocket.CAM_A)
        }
        file_calibration = os.path.join(output_dir, f'{thread}_calibration.json')
        with open(file_calibration, 'w') as f:
            json.dump(calib_dict, f, indent=4)

        file_color = open(os.path.join(output_dir, f'{thread}_color.h265'), 'wb')
        file_monoL = open(os.path.join(output_dir, f'{thread}_monoL.h264'), 'wb')
        file_monoR = open(os.path.join(output_dir, f'{thread}_monoR.h264'), 'wb')
        file_imus = open(os.path.join(output_dir, f'{thread}_imu_data.txt'), 'wb')
        print(f'{thread} started...')
        try:
            while True:
                imu_message = imu_queue.get()
                for imu_packet in imu_message.packets:
                    acceleroValues = imu_packet.acceleroMeter
                    gyroValues = imu_packet.gyroscope
                    acceleroTs = acceleroValues.getTimestampDevice()
                    gyroTs = gyroValues.getTimestampDevice()

                    imu_data = {
                        "acceleroMeter" : {
                            "x": acceleroValues.x,
                            "y": acceleroValues.y,
                            "z": acceleroValues.z,
                            "timestamp": acceleroTs
                        },
                        "gyroscope"     : {
                            "x": gyroValues.x,
                            "y": gyroValues.y,
                            "z": gyroValues.z,
                            "timestamp": gyroTs
                        }
                    }
                    file_imus.write(("{'imu_timestamp': '" + str(imu_message.getTimestampDevice()) + "',    'IMU': " + str(imu_data) + '\n').encode())

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