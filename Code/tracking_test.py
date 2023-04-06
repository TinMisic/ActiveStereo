import cv2 as cv
import motion_detection as md
import servo_controller as servo
import camera
import time
import sys

# define global state and constants
state = 'init'
angles = [0, 0, 0, 0] # current servo angles

# initialize and start camera threads
left = camera.Camera("camera_parameters/camera0_intrinsics.dat",6)
right = camera.Camera("camera_parameters/camera1_intrinsics.dat",0)

# servo constants
scaling = 180/270 
offset = 135    # center offset
offsets = [x+offset for x in [2, 7, -4, -4]]    # individual offsets
limit = 90      # absolute limit of servo movement
speed = 60/0.13 # degrees per second

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
    prev_frame_l = left.get_frame()
    prev_frame_r = right.get_frame()
    flag = True

    old_centroid_l = (None, None)
    old_centroid_r = (None, None)
    while(flag):
        curr_frame_l = left.get_frame()
        curr_frame_r = right.get_frame()

        threshold_l, centroid_l = md.centroidOfDif(prev_frame_l, curr_frame_l, 20000)
        threshold_r, centroid_r = md.centroidOfDif(prev_frame_r, curr_frame_r, 20000)

        centroid_l = md.centroidSmoothing(old_centroid_l, centroid_l, 0.85)
        centroid_r = md.centroidSmoothing(old_centroid_r, centroid_r, 0.85)

        if centroid_l != (None, None) and centroid_r != (None, None):
            if( old_centroid_l!=(None,None) and ((old_centroid_l[0] - centroid_l[0])**2+(old_centroid_l[1]-centroid_l[1])**2)**0.5 < 10):
                flag=False

            rel_ang = servo.getAngles(centroid_l, centroid_r, left.intrinsic, right.intrinsic)
            print(centroid_l, centroid_r)
            print(rel_ang)

        prev_frame_l = curr_frame_l
        old_centroid_l = centroid_l
        prev_frame_r = curr_frame_r
        old_centroid_r = centroid_r
        cv.imshow("left", curr_frame_l)
        cv.imshow("right",curr_frame_r)
        if cv.waitKey(1) == ord('q'):
            raise KeyboardInterrupt
    
    print("Tracking stopped")
    print(old_centroid_l,old_centroid_r)
    for i in range(len(servos)):
        servos[i].write((rel_ang[i] + offsets[i])*scaling)

    while(True):
        curr_frame_l = left.get_frame()
        curr_frame_r = right.get_frame()
        cv.imshow("left", curr_frame_l)
        cv.imshow("right",curr_frame_r)
        if cv.waitKey(1) == ord('q'):
            raise KeyboardInterrupt

except KeyboardInterrupt as e:
    for i in range(len(servos)):
        servos[i].write((offsets[i])*scaling)
        
    time.sleep(1)
    cv.destroyAllWindows()
    left.stop()
    right.stop()
    board.exit()