import cv2 as cv
import motion_detection as md
import servo_controller as servo
import util
import camera
import time
import signal
import sys

def add_ang(angles, rel_ang, f = 0.05):
    return [(f*angles[i] + (1-f)*(angles[i]+rel_ang[i])) for i in range(len(angles))]

# signal handling - debug stop sends SIGTERM, but that doesn't release the cameras and causes problems
class TerminationSignal(Exception):
    pass

def handle_sigterm(signum, frame):
    raise TerminationSignal("Received SIGTERM signal, terminating...")

signal.signal(signal.SIGTERM, handle_sigterm)

# define global state and constants
state = 'init'
angles = [0, 0, 0, 0] # current servo angles

# initialize image mapping, edge detector and shift operator
mp = util.IdentityMap()
det = util.SobelCartesian()
shft = util.ShiftCartesian()

# initialize and start camera threads
left = camera.Camera("camera_parameters/camera0_intrinsics.dat",6)
right = camera.Camera("camera_parameters/camera1_intrinsics.dat",0)

# servo constants
scaling = 180/270 
offset = 135    # center offset
offsets = [x+offset for x in [2, 7, -4, -4]]    # individual offsets
limit = 90      # absolute limit of servo movement
speed = 60/0.13 # degrees per second

thresh = 10000

# start board and initialize servos
print("INFO: Initializing servos...")
import pyfirmata
board = pyfirmata.Arduino('/dev/ttyUSB0')

servos = list() # order = left-lower, left-upper, right-lower, right-upper
for i in range(4):
    s = board.get_pin('d:'+str(2+i)+':s') # initialize servo
    s.write(offsets[i]*scaling) # home position
    servos.append(s)

print("INFO: Start tracking")
# main loop
try:
    # get frames, map them and detect edges
    previous_r = right.get_frame()#det.detect(mp.map(right.get_frame()))
    previous_l = left.get_frame()#det.detect(mp.map(left.get_frame()))

    print("Before while")
    while True:
        # cv.imshow("L",previous_l)
        # cv.imshow("R",previous_r)
        # get frames, map them and detect edges
        current_r = right.get_frame()#det.detect(mp.map(right.get_frame()))
        current_l = left.get_frame()#det.detect(mp.map(left.get_frame()))
        # print("State case")
        if state=='init':
            # retrieve centroid of movement for left
            _, centroidL = md.centroidOfDif(previous_l, current_l)
            # retrieve centroid of movement for right
            _, centroidR = md.centroidOfDif(previous_r, current_r)
            
            print(centroidL,centroidR)
            # input()
            # if centroid is detected, move cameras and change state
            if centroidL!=(None, None) and centroidR!=(None, None):
                # print("INFO: Movement...")
                print("------------------------")
                print("angles",angles)
                rel_ang = servo.getAngles(centroidL, centroidR, left.intrinsic, right.intrinsic) # new relative angles
                print("rel_ang",rel_ang)
                # angles = [min(angles[i] + rel_ang[i], limit) if angles[i] + rel_ang[i] > 0 else max(angles[i] + rel_ang[i], -limit) for i in range(len(angles))] # new absolute angles
                angles = [angles[i] + rel_ang[i] for i in range(len(angles))]
                # angles = add_ang(angles,rel_ang)
                print("angles+rel_ang",angles)
                max_dif = abs(max(rel_ang, key = lambda x:abs(x))) # get max servo shift
                wait = max_dif/speed # get waiting time for servos to arrive - no encoder so I'm relying on the speed of the servos. Waiting until cameras are still to capture new frame

                # input("Enter to move servos")
                for i in range(len(servos)):
                    servos[i].write((angles[i]+offsets[i])*scaling)
                print(angles)
                sys.stdout.flush()
            
                time.sleep(wait*10) # wait for sevos to arrive
                # state = "tracking"
            
            previous_l = current_l
            previous_r = current_r

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
except (KeyboardInterrupt, TerminationSignal) as e:
    for i in range(len(servos)):
        servos[i].write((offsets[i])*scaling)
        
    time.sleep(1)
    cv.destroyAllWindows()
    left.stop()
    right.stop()
    board.exit()