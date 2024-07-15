import cv2
import os
import time
import argparse
import threading
import depthai as dai
from utils.host_sync import HostSync
from utils.general_helper import save_calibration, get_queues, get_files, write_imu

def main(thread, output_dir):
    pipeline = dai.Pipeline()

    monoL_pipeline = pipeline.create(dai.node.MonoCamera)
    monoL_pipeline.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    monoL_pipeline.setBoardSocket(dai.CameraBoardSocket.CAM_B)

    monoR_pipeline = pipeline.create(dai.node.MonoCamera)
    monoR_pipeline.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    monoR_pipeline.setBoardSocket(dai.CameraBoardSocket.CAM_C)

    depth_pipeline = pipeline.create(dai.node.StereoDepth)
    depth_pipeline.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    depth_pipeline.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
    depth_pipeline.setLeftRightCheck(True)
    depth_pipeline.setExtendedDisparity(False)
    depth_pipeline.setSubpixel(False)
    monoL_pipeline.out.link(depth_pipeline.left)
    monoR_pipeline.out.link(depth_pipeline.right)
    depth_pipeline.setDepthAlign(dai.CameraBoardSocket.CAM_A)

    xout_depth = pipeline.create(dai.node.XLinkOut)
    xout_depth.setStreamName("depth")
    depth_pipeline.depth.link(xout_depth.input)

    color_pipeline = pipeline.create(dai.node.ColorCamera)
    color_pipeline.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    color_pipeline.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    color_pipeline.setFps(30)

    color_encoder = pipeline.create(dai.node.VideoEncoder)
    color_encoder.setDefaultProfilePreset(30, dai.VideoEncoderProperties.Profile.H265_MAIN)
    color_pipeline.video.link(color_encoder.input)

    xout_color = pipeline.create(dai.node.XLinkOut)
    xout_color.setStreamName('color')
    color_encoder.bitstream.link(xout_color.input)

    monoL_encoder = pipeline.create(dai.node.VideoEncoder)
    monoL_encoder.setDefaultProfilePreset(monoL_pipeline.getFps(), dai.VideoEncoderProperties.Profile.H264_MAIN)
    monoL_pipeline.out.link(monoL_encoder.input)

    monoR_encoder = pipeline.create(dai.node.VideoEncoder)
    monoR_encoder.setDefaultProfilePreset(monoR_pipeline.getFps(), dai.VideoEncoderProperties.Profile.H264_MAIN)
    monoR_pipeline.out.link(monoR_encoder.input)

    xout_monoL = pipeline.create(dai.node.XLinkOut)
    xout_monoL.setStreamName('monoL')
    monoL_encoder.bitstream.link(xout_monoL.input)

    xout_monoR = pipeline.create(dai.node.XLinkOut)
    xout_monoR.setStreamName('monoR')
    monoR_encoder.bitstream.link(xout_monoR.input)

    imu_pipeline = pipeline.create(dai.node.IMU)
    xout_imu   = pipeline.createXLinkOut()
    xout_imu.setStreamName("imu")
    imu_pipeline.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 120)
    imu_pipeline.setBatchReportThreshold(1)
    imu_pipeline.setMaxBatchReports(30)
    imu_pipeline.out.link(xout_imu.input)

    os.makedirs(output_dir+f'/{thread}_depth')
    with dai.Device(pipeline) as device:
        queues, imu_queue = get_queues(device)
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