import depthai as dai


def get_pipeline():
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
    depth_pipeline.setLeftRightCheck(False)
    depth_pipeline.setExtendedDisparity(False)

    depth_pipeline.setSubpixel(False)
    monoL_pipeline.out.link(depth_pipeline.left)
    monoR_pipeline.out.link(depth_pipeline.right)

    encoder_depth = pipeline.create(dai.node.VideoEncoder)
    encoder_depth.setDefaultProfilePreset(monoL_pipeline.getFps(), dai.VideoEncoderProperties.Profile.MJPEG)
    depth_pipeline.disparity.link(encoder_depth.input)

    xout_depth = pipeline.create(dai.node.XLinkOut)
    xout_depth.setStreamName("depth")
    encoder_depth.bitstream.link(xout_depth.input)

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
    imu_pipeline.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 500)
    imu_pipeline.setBatchReportThreshold(1)
    imu_pipeline.setMaxBatchReports(30)
    imu_pipeline.out.link(xout_imu.input)

    return pipeline