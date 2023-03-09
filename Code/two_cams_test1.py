# Simultaneous stream works, but there is a delay between left and right frame

import cv2 as cv
import numpy as np

cap0 = cv.VideoCapture(4)
if not cap0.isOpened(): print("Cannot open camera 0")
cap1 = cv.VideoCapture(6)
if not cap1.isOpened(): print("Cannot open camera 1")

if not cap0.isOpened() or not cap1.isOpened(): exit()

while True:
    ret0, frame0 = cap0.read()
    print(frame0.shape)
    ret1, frame1 = cap1.read()
    print(frame1.shape)

    if not ret0 or not ret1:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    
    f0 = cv.resize(frame0, (frame1.shape[1],frame1.shape[0]),interpolation=cv.INTER_AREA)
    together = np.hstack((frame1,f0))
    cv.imshow('frame',together)
    if cv.waitKey(1) == ord('q'):
        break

cap0.release()
cap1.release()
cv.destroyAllWindows()