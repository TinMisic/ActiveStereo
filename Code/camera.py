import cv2
import numpy as np
import threading

class Camera(threading.Thread):
    '''Camera thread'''

    def __init__(self, calib_file, device_index):
        super(Camera, self).__init__()
        self.device_index = device_index
        self.frame = None
        self.new_frame_condition = threading.Condition()
        self.capture_event = threading.Event()
        self._stop_event = threading.Event()

        self.parse_calibration_file(calib_file)
        self.start()

    def run(self):
        cap = cv2.VideoCapture(self.device_index)
        while not self._stop_event.is_set():
            # Wait for a request to capture a new frame
            self.capture_event.wait()
            # Reset the event
            self.capture_event.clear()

            # Capture a new frame from the camera
            ret, frame = cap.read()
            if ret:
                # Acquire the condition lock and set the new frame
                with self.new_frame_condition:
                    self.frame = cv2.undistort(frame, self.intrinsic, self.distortion)
                    self.new_frame_condition.notify()
        cap.release()
        print("INFO: Camera "+str(self.device_index)+" released.")

    def stop(self):
        self._stop_event.set()
        self.capture_event.set()

    def request_frame(self):
        # Set the capture event to request a new frame
        self.capture_event.set()

    def get_frame(self):
        # Acquire the condition lock
        with self.new_frame_condition:
            # Wait for a new frame to be set by the camera thread
            self.new_frame_condition.wait()
            # Return the most recent frame captured by the camera
            return self.frame

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
    # Camera test
    cam = Camera("camera_parameters/camera0_intrinsics.dat", 4)

    while True:
        cam.request_frame()

        frame = cam.get_frame()

        cv2.imshow('Camera',frame)
        if cv2.waitKey(1) == ord('q'):
            cam.stop()
            break

    print("main stopped")