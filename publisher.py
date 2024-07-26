import cv2
import numpy as np
import pickle
import os
import depthai as dai
import zmq
from modules.recording.encoders import *
from modules.recording.pipeline import *
from modules.recording.xouts import *
from utils.general_helper import save_calibration, get_queues
from utils.host_sync import HostSync

def create_pipeline():
    pipeline = dai.Pipeline()

    monoL_pipeline = get_mono_pipeline(pipeline, dai.CameraBoardSocket.CAM_B)
    monoR_pipeline = get_mono_pipeline(pipeline, dai.CameraBoardSocket.CAM_C)
    depth_pipeline = get_depth_pipeline(pipeline, monoL_pipeline, monoR_pipeline)
    color_pipeline = get_color_pipeline(pipeline)
    imu_pipeline   = get_imu_pipeline(pipeline)

    color_encoder = get_encoder(pipeline, dai.VideoEncoderProperties.Profile.H265_MAIN)
    monoL_encoder = get_encoder(pipeline, dai.VideoEncoderProperties.Profile.H264_MAIN)
    monoR_encoder = get_encoder(pipeline, dai.VideoEncoderProperties.Profile.H264_MAIN)

    color_pipeline.video.link(color_encoder.input)
    monoL_pipeline.out.link(monoL_encoder.input)
    monoR_pipeline.out.link(monoR_encoder.input)

    set_depth_xout(pipeline, depth_pipeline)
    xout_color = get_xout(pipeline, 'color')
    xout_monoL = get_xout(pipeline, 'monoL')
    xout_monoR = get_xout(pipeline, 'monoR')
    xout_imu   = get_xout(pipeline, 'imu')

    color_encoder.bitstream.link(xout_color.input)
    monoL_encoder.bitstream.link(xout_monoL.input)
    monoR_encoder.bitstream.link(xout_monoR.input)

    imu_pipeline.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 120)
    imu_pipeline.setBatchReportThreshold(1)
    imu_pipeline.setMaxBatchReports(30)
    imu_pipeline.out.link(xout_imu.input)

    return pipeline

def main(pipeline, output_dir):
    context = zmq.Context()
    color_pub = context.socket(zmq.PUB)
    monoL_pub = context.socket(zmq.PUB)
    monoR_pub = context.socket(zmq.PUB)
    imu_pub = context.socket(zmq.PUB)

    color_pub.bind("tcp://*:5555")
    monoL_pub.bind("tcp://*:5556")
    monoR_pub.bind("tcp://*:5557")
    imu_pub.bind("tcp://*:5558")

    os.makedirs(output_dir, exist_ok=True)

    with dai.Device(pipeline) as device:
        queues, imu_queue = get_queues(device)
        sync = HostSync()
        save_calibration(device.readCalibration(), output_dir, "depthai")
        print("Publisher started...")

        try:
            while True:
                for queue in queues:
                    message = sync.add_msg(queue.getName(), queue.get())
                    if message:
                        color_buffer = pickle.dumps(message['color'].getData())
                        monoL_buffer = pickle.dumps(message['monoL'].getData())
                        monoR_buffer = pickle.dumps(message['monoR'].getData())
                        depth_buffer = pickle.dumps(message['depth'].getFrame())

                        color_pub.send(color_buffer)
                        monoL_pub.send(monoL_buffer)
                        monoR_pub.send(monoR_buffer)
                        imu_pub.send(depth_buffer)
        except KeyboardInterrupt:
            print("Publisher stopped.")

if __name__ == "__main__":
    pipeline = create_pipeline()
    output_dir = "output"
    main(pipeline, output_dir)
