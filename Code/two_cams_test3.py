import cv2
import numpy as np
import threading
from queue import Queue

endflag = False

class CameraThread(threading.Thread):
    def __init__(self, threadID, name, cameraID, queue):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.camera = cv2.VideoCapture(cameraID)
        self.queue = queue

    def run(self):
        print("Starting " + self.name)
        while True:
            ret, frame = self.camera.read()
            if not ret or endflag:
                break
            self.queue.put((self.threadID, frame))
        self.camera.release()

def main():
    global endflag
    # Create a queue to hold the frames from the camera threads
    queue = Queue()

    # Create two threads for two cameras
    thread1 = CameraThread(1, "Camera 1", 4, queue)
    thread2 = CameraThread(2, "Camera 2", 6, queue)

    # Start the threads
    thread1.start()
    thread2.start()

    # Create a window to display the combined frames
    cv2.namedWindow("Combined Frames", cv2.WINDOW_NORMAL)

    # Initialize the combined frames as an empty numpy array
    left = np.zeros((480,640,3))
    right = np.zeros((480,640,3))
    combined = np.zeros((480,640,3))

    # Loop to display the frames from the queue
    while True:
        # Get a frame from the queue
        item = queue.get()
        threadID, frame = item

        # Display the frame in the appropriate window
        #cv2.imshow("Camera " + str(threadID), frame)

        # Combine the frames and display in the combined frames window
        if threadID == 1:
            left = frame
        else:
            right = frame
            combined = np.hstack([left, right])
            cv2.imshow("Combined Frames", combined)

        # Exit if 'q' is pressed
        if cv2.waitKey(1) == ord('q'):
            endflag = True
            break

    # Wait for the threads to finish
    thread1.join()
    thread2.join()

    # Close all windows
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
