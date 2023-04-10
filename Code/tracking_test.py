import cv2 as cv
import motion_detection as md
import servo_controller as servo
import camera
import time
import numpy as np
import util

# define global state and constants
angles = [0, 0, 0, 0] # current servo angles
init_flag = True
timeout = 5 # seconds of immobility after which to start initilizaton again

# initialize and start camera threads
left = camera.Camera("camera_parameters/camera0_intrinsics.dat",4)
right = camera.Camera("camera_parameters/camera1_intrinsics.dat",6)

# define mapping and operators
mp = util.IdentityMap()#util.LogPolarMap((480, 640),(300, 100))
det = util.SobelCartesian()#util.SobelPseudoLPM(mp)
shft = util.ShiftCartesian()#util.ShiftPseudoLPM(mp)
get_edges = lambda x: det.detect(mp.map(x))
movement_thresh = 200000 # empirical movement threshold > Identity=200000, LPM=2000

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

print("INFO: Begin procedure")
# main loop
try:
    while True:
        if init_flag:
            print("INFO: Initialization starting")
            prev_frame_l = get_edges(left.get_frame())
            prev_frame_r = get_edges(right.get_frame())
            old_centroid_l = (None, None)
            old_centroid_r = (None, None)
            while(init_flag):
                curr_frame_l = get_edges(left.get_frame())
                curr_frame_r = get_edges(right.get_frame())

                threshold_l, centroid_l = md.centroidOfDif(prev_frame_l, curr_frame_l, movement_thresh)
                threshold_r, centroid_r = md.centroidOfDif(prev_frame_r, curr_frame_r, movement_thresh)

                centroid_l = md.centroidSmoothing(old_centroid_l, centroid_l, 0.85)
                centroid_r = md.centroidSmoothing(old_centroid_r, centroid_r, 0.85)

                if centroid_l != (None, None) and centroid_r != (None, None):
                    if( old_centroid_l!=(None,None) and ((old_centroid_l[0] - centroid_l[0])**2+(old_centroid_l[1]-centroid_l[1])**2)**0.5 < 10):
                        init_flag=False

                        rel_ang = servo.getAngles(centroid_l, centroid_r, left.intrinsic, right.intrinsic)
                        for i in range(len(angles)):
                            angles[i]+=rel_ang[i]
                        # print(centroid_l, centroid_r)
                        # print(rel_ang)
                        # print(angles)

                prev_frame_l = curr_frame_l
                old_centroid_l = centroid_l
                prev_frame_r = curr_frame_r
                old_centroid_r = centroid_r

                combined = np.hstack((mp.inv(curr_frame_l),mp.inv(curr_frame_r)))
                cv.imshow("view", combined)
                if cv.waitKey(1) == ord('q'):
                    raise KeyboardInterrupt
            
            print("INFO: Initialization stopped")
            # print(old_centroid_l,old_centroid_r)
            # print("Moving servos")
            for i in range(len(servos)):
                servos[i].write((angles[i] + offsets[i])*scaling)

        else:
            print("INFO: Tracking started")
            still_start = time.time()
            while not init_flag:
                if (time.time() - still_start >= timeout):
                    print("INFO: No movement in",timeout,"seconds.")
                    break
                curr_frame_l = get_edges(left.get_frame())
                curr_frame_r = get_edges(right.get_frame())

                # generate shifts of vertical edge images
                shiftDict = util.generateShifts(curr_frame_r, shft)

                # ZERO DISPARITY FILTER
                # convert left to binary image
                _, left_binary = cv.threshold(curr_frame_l,50, 255,cv.THRESH_BINARY)

                ZDFDict = dict()
                for k in shiftDict.keys():
                    # convert right shift to binary image
                    _, right_binary = cv.threshold(shiftDict[k],50,255,cv.THRESH_BINARY)
                    # apply AND etween left and right
                    ZDFDict[k] = cv.bitwise_and(left_binary, right_binary)

                # get pixels from ZDF shift with max match
                pixel_shift = util.getMaxZDF(ZDFDict)

                # convert pixel to angle
                ZDFangle = servo.getAngleNormalised(right.intrinsic, pixel_shift)

                # CENTROID CALCULATION
                threshold_l, centroid_l = md.centroidOfDif(prev_frame_l, curr_frame_l, movement_thresh)
                threshold_r, centroid_r = md.centroidOfDif(prev_frame_r, curr_frame_r, movement_thresh)

                centroid_l = md.centroidSmoothing(old_centroid_l, centroid_l, 0.85)
                centroid_r = md.centroidSmoothing(old_centroid_r, centroid_r, 0.85)

                if centroid_l != (None, None) and centroid_r != (None, None):
                    rel_ang = servo.getAngles(centroid_l, centroid_r, left.intrinsic, right.intrinsic)
                    for i in range(len(angles)):
                        angles[i]+=rel_ang[i]
                    # print(centroid_l, centroid_r)
                    # print(rel_ang)
                    # print(angles)

                # Add virtual horopter angle to correct actual horopter
                angles[2]+=ZDFangle

                if pixel_shift!=0: # reset still timer
                    still_start = time.time()

                for i in range(len(servos)):
                    servos[i].write((angles[i] + offsets[i])*scaling) # move servos

                # Wait for servos to arrive?

                prev_frame_l = curr_frame_l
                old_centroid_l = centroid_l
                prev_frame_r = curr_frame_r
                old_centroid_r = centroid_r                

                combined = np.hstack((mp.inv(curr_frame_l),mp.inv(curr_frame_r)))
                cv.imshow("view", combined)
                if cv.waitKey(1) == ord('q'):
                    raise KeyboardInterrupt
            
            init_flag = True

except KeyboardInterrupt as e:
    for i in range(len(servos)):
        servos[i].write((offsets[i])*scaling)
        
    time.sleep(1)
    cv.destroyAllWindows()
    left.stop()
    right.stop()
    board.exit()