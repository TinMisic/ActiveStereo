import cv2 as cv
import motion_detection as md
import servo_controller as servo

# define global state and constants
state = 'init'

# initialize image mapping, edge detector and shift operator
pass

# initialize camera threads and servos
pass 

# main loop
try:
    # fetch initial stereo images
    pass

    while True:
        # get new stereo images
        pass

        if state=='init':
            # retrieve centroid of movement
            pass
            # if centroid is detected, move cameras and change state
            pass
        elif state=='tracking':
            # edge detection
            pass
            # create RIGHT virtual horopters by shift operator
            pass
            # AND operation for Zero Disparity Filter
            pass
            # maximal matching
            pass
            # calculate centroid and move cameras
            pass
except KeyboardInterrupt:
    cv.destroyAllWindows()