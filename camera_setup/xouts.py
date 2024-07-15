import depthai as dai

def set_depth_xout(pipeline, depth_pipeline):
    xout_depth = pipeline.create(dai.node.XLinkOut)
    xout_depth.setStreamName("depth")
    depth_pipeline.depth.link(xout_depth.input)