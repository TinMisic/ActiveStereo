import cv2
import numpy as np
import threading
import time

class Camera(threading.Thread):
    '''Camera thread'''

    def __init__(self, calib_file, device_index):
        super(Camera, self).__init__()
        self.device_index = device_index
        self.frame = None
        self._stop_event = threading.Event()
        self.buffer = []
        self.buffer_lock = threading.Lock()

        self.parse_calibration_file(calib_file)
        self.start()

    def run(self):
        print("INFO: Start camera thread",self.device_index)
        cap = cv2.VideoCapture(self.device_index)
        while not self._stop_event.is_set():
            ret, frame = cap.read()
            if ret:
                # Undistort the frame
                frame = cv2.undistort(cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY), self.intrinsic, self.distortion)

                # Append the latest frame to the buffer
                with self.buffer_lock:
                    self.buffer.append(frame)

                # If the buffer has more than 1 frame, discard the oldest frame
                with self.buffer_lock:
                    if len(self.buffer) > 1:
                        self.buffer.pop(0)

            else:
                print("ERROR: Frame capture error in Camera",self.device_index)
                # time.sleep(0.1)

        cap.release()
        print("INFO: Camera "+str(self.device_index)+" released.")

    def stop(self):
        self._stop_event.set()

    def get_frame(self):
        # Get the latest frame from the buffer
        with self.buffer_lock:
            if len(self.buffer) > 0:
                return self.buffer[-1]
            else:
                return None


    def parse_calibration_file(self, file_path):
        """Parses a calibration file and returns the intrinsic matrix and distortion parameters.
        """
        with open(file_path, 'r') as f:
            lines = f.readlines()

        intrinsic = []
        distortion = []
        for i, line in enumerate(lines):
            if line.startswith('intrinsic'):
                intrinsic = np.array([float(x) for x in lines[i+1].split()]).reshape(3,3)
            elif line.startswith('distortion'):
                distortion = np.array([float(x) for x in lines[i+1].split()])

        self.intrinsic = intrinsic
        self.distortion = distortion

if __name__=="__main__":
    cam1 = Camera("camera_parameters/camera0_intrinsics.dat", 0)
    cam2 = Camera("camera_parameters/camera1_intrinsics.dat", 6)

    while True:
        # # Request new frames from all cameras
        # cam1.get_frame()
        # cam2.get_frame()

        # # Get the latest frames from each camera
        frame1 = cam1.get_frame()
        if frame1 is not None:
            # print(frame1.shape)
            cv2.imshow('L',frame1)

        frame2 = cam2.get_frame()
        if frame2 is not None:
            cv2.imshow('R', frame2)

        if cv2.waitKey(1) == ord('q'):
            cam1.stop()
            cam2.stop()
            break

    print("main stopped")
