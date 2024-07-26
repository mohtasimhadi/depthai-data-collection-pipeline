import cv2
import pickle
import zmq
import numpy as np

def main():
    context = zmq.Context()
    color_sub = context.socket(zmq.SUB)
    monoL_sub = context.socket(zmq.SUB)
    monoR_sub = context.socket(zmq.SUB)
    imu_sub = context.socket(zmq.SUB)

    color_sub.connect("tcp://localhost:5555")
    monoL_sub.connect("tcp://localhost:5556")
    monoR_sub.connect("tcp://localhost:5557")
    imu_sub.connect("tcp://localhost:5558")

    color_sub.setsockopt_string(zmq.SUBSCRIBE, "")
    monoL_sub.setsockopt_string(zmq.SUBSCRIBE, "")
    monoR_sub.setsockopt_string(zmq.SUBSCRIBE, "")
    imu_sub.setsockopt_string(zmq.SUBSCRIBE, "")

    while True:
        color_data = pickle.loads(imu_sub.recv())
        cv2.imshow("Depth", color_data)

        if cv2.waitKey(1) == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
