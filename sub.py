import cv2
import pickle
import zmq
import numpy as np

def main():
    context = zmq.Context()
    subscriber = context.socket(zmq.SUB)

    subscriber.connect("tcp://localhost:5555")

    subscriber.setsockopt_string(zmq.SUBSCRIBE, "")
    while True:
        message = pickle.loads(subscriber.recv())
        print(message['imu'])
if __name__ == "__main__":
    main()
