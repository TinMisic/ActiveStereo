import cv2 as cv
import motion_detection as md
import servo_controller as servo
import util
import camera

# define global state and constants
state = 'init'

# initialize image mapping, edge detector and shift operator
mp = util.IdentityMap()
det = util.SobelCartesian()
shft = util.ShiftCartesian()

# initialize and start camera threads
left = camera.Camera("camera_parameters/camera0_intrinsics.dat",4)
right = camera.Camera("camera_parameters/camera1_intrinsics.dat",6)

# servo constants
scaling = 180/270 
offset = 135    # center offset
offsets = [x+offset for x in [2, 7, -4, -4]]    # individual offsets
speed = 60/0.13 # degrees per second

# start board and initialize servos
print("INFO: Initializing servos...")
import pyfirmata
board = pyfirmata.Arduino('/dev/ttyUSB0')

servos = list() # order = left-lower, left-upper, right-lower, right-upper
for i in range(4):
    s = board.getboard.get_pin('d:'+str(2+i)+':s') # initialize servo
    s.write(offsets[i]*scaling) # home position
    servos.append(s)

print("INFO: Start tracking")
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
    left.stop()
    right.stop()
    board.exit()