import os, json
import depthai as dai
from camera_setup.pipeline import get_pipeline
from utils.host_sync import HostSync

output_dir = 'depthai_output'
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

pipeline = get_pipeline()

with dai.Device(pipeline) as device:
    queues = []
    queues.append(device.getOutputQueue(name="depth", maxSize=30, blocking=True))
    queues.append(device.getOutputQueue(name="color", maxSize=30, blocking=True))
    queues.append(device.getOutputQueue(name="monoL", maxSize=30, blocking=True))
    queues.append(device.getOutputQueue(name="monoR", maxSize=30, blocking=True))
    queues.append(device.getOutputQueue(name="imu", maxSize=30, blocking=True))
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
    file_calibration = os.path.join(output_dir, 'calibration.json')
    with open(file_calibration, 'w') as f:
        json.dump(calib_dict, f, indent=4)

    imu_list = []
    # baseTs = None

    file_color = open(os.path.join(output_dir, 'color.h265'), 'wb')
    file_monoL = open(os.path.join(output_dir, 'monoL.h264'), 'wb')
    file_monoR = open(os.path.join(output_dir, 'monoR.h264'), 'wb')
    file_depth = open(os.path.join(output_dir, 'depth.mjpeg'), 'wb')
    file_times = open(os.path.join(output_dir, 'timestamps.txt'), 'wb')
    file_imus = open(os.path.join(output_dir, 'imu_data.txt'), 'wb')
    print("Press Ctrl+C to stop encoding...")
    try:
        while True:
            imu_message = imu_queue.get()
            if imu_message is not None:
                imu_packets = imu_message.packets
                for imu_packet in imu_packets:
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
                    file_depth.write(message['depth'].getData())
                    file_color.write(message['color'].getData())
                    file_monoL.write(message['monoL'].getData())
                    file_monoR.write(message['monoR'].getData())
                    file_times.write(("{'camera_timestamp': '" + str(message['depth'].getTimestampDevice()) + "', 'frame_number': " + str(message['depth'].getSequenceNum())+"}\n").encode())
    except KeyboardInterrupt:
        pass