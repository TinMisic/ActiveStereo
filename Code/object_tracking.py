import cv2 as cv
import motion_detection as md
import servo_controller as servo
import camera
import time
import numpy as np
import util
import datetime

starttime = time.time()

# define global state and constants
angles = [0, 0, 0, 0] # current servo angles
init_flag = True
timeout = 1 # seconds of immobility after which to start initilizaton again

# initialize and start camera threads
left = camera.Camera("camera_parameters/camera0_intrinsics.dat",4)
right = camera.Camera("camera_parameters/camera1_intrinsics.dat",6)

# define mapping and operators
mp = util.LogPolarMap((640, 480),(300, 100)) #util.IdentityMap() #
det = util.SobelPseudoLPM(mp) #util.SobelCartesian()#
shft = util.ShiftPseudoLPM(mp) #util.ShiftCartesian()#
get_edges = lambda x: det.detect(mp.map(x))
movement_thresh = 2000 # empirical movement threshold > Identity=200000, LPM=2000

# servo constants
scaling = 180/270 
offset = 135    # center offset
offsets = [x+offset for x in [2, 7, -4, -4]]    # individual offsets
limit = 90      # absolute limit of servo movement
speed = 60/0.13 # degrees per second

# Define the ROI
x, y, w, h = 20, 20, 600, 440 # x, y, width, height
roi = np.zeros((480, 640), dtype=np.uint8)
cv.rectangle(roi, (x, y), (x + w, y + h), (255, 255, 255), -1)

# Define kernel for dilation
dim = 9
d_kernel = np.ones((dim,dim), np.uint8)
e_kernel = np.ones((2,2),np.uint8)

# start board and initialize servos
print("INFO: Initializing servos...")
import pyfirmata
board = pyfirmata.Arduino('/dev/ttyUSB0')

servos = list() # order = left-lower, left-upper, right-lower, right-upper
for i in range(4):
    s = board.get_pin('d:'+str(2+i)+':s') # initialize servo
    s.write(offsets[i]*scaling) # home position
    servos.append(s)

# open log file
now = datetime.datetime.now()
filename = "log/" + now.strftime("%Y-%m-%d-%H-%M-%S") + ".txt"
log = open(filename,'w')
lost_counter=0 

# open video file
# Define the codec and create a VideoWriter object
videoname = "log/" + now.strftime("%Y-%m-%d-%H-%M-%S") + ".mp4"
fourcc = cv.VideoWriter_fourcc(*'mp4v')  # use appropriate codec
fps = 30.0  # frames per second
vwidth = 3*640  # frame width
vheight = 480  # frame height
vlog = cv.VideoWriter(videoname, fourcc, fps, (vwidth, vheight))

time.sleep(1)
print("INFO: Begin procedure")
# main loop
try:
    while True:
        if init_flag:
            print("INFO: Initialization starting")
            prev_frame_l = curr_frame_l = get_edges(left.get_frame())
            prev_frame_r = curr_frame_r = get_edges(right.get_frame())
            old_centroid_l = (None, None)
            old_centroid_r = (None, None)
            while(init_flag):
                curr_frame_l = get_edges(left.get_frame())
                curr_frame_r = get_edges(right.get_frame())

                threshold_l, centroid_l = md.centroidOfDif(mp.inv(prev_frame_l), mp.inv(curr_frame_l), movement_thresh)
                threshold_r, centroid_r = md.centroidOfDif(mp.inv(prev_frame_r), mp.inv(curr_frame_r), movement_thresh)

                centroid_l = md.centroidSmoothing(old_centroid_l, centroid_l, 0.85)
                centroid_r = md.centroidSmoothing(old_centroid_r, centroid_r, 0.85)

                if centroid_l != (None, None) and centroid_r != (None, None):
                    if( old_centroid_l!=(None,None) and ((old_centroid_l[0] - centroid_l[0])**2+(old_centroid_l[1]-centroid_l[1])**2)**0.5 < 10):
                        init_flag=False

                        rel_ang = servo.getAngles(centroid_l, centroid_r, left.intrinsic, right.intrinsic)
                        angles = util.updateAngles(angles,rel_ang)

                prev_frame_l = curr_frame_l
                old_centroid_l = centroid_l
                prev_frame_r = curr_frame_r
                old_centroid_r = centroid_r

                combined = np.hstack((mp.inv(curr_frame_l),mp.inv(curr_frame_r)))
                cv.imshow("view", combined)
                if cv.waitKey(1) == ord('q'):
                    raise KeyboardInterrupt
            
            print("INFO: Initialization stopped")
            for i in range(len(servos)):
                servos[i].write((angles[i] + offsets[i])*scaling)

        else:
            print("INFO: Tracking started")
            still_start = time.time()
            while not init_flag:
                if (time.time() - still_start >= timeout):
                    print("INFO: No movement in",timeout,"seconds.")
                    lost_counter+=1
                    break

                l = mp.map(left.get_frame())
                r = mp.map(right.get_frame())
                curr_frame_l = det.detect(l)
                curr_frame_r = det.detect(r)

                # generate shifts of vertical edge images
                shiftDict = util.generateShifts(curr_frame_r, shft,shifts=45,stride=1)

                # ZERO DISPARITY FILTER
                thresh = 100 # tested and is somewhat good

                # convert left to binary image
                _, left_binary = cv.threshold(curr_frame_l,thresh, 255,cv.THRESH_BINARY)
                # dilate left_binary
                # left_binary = cv.erode(left_binary,e_kernel,iterations=1)
                left_binary = mp.map(cv.dilate(mp.inv(left_binary),d_kernel,iterations=1))

                ZDFDict = dict()
                for k in shiftDict.keys():
                    # convert right shift to binary image
                    _, right_binary = cv.threshold(shiftDict[k],thresh,255,cv.THRESH_BINARY)
                    # dilate right_binary
                    # right_binary = cv.erode(right_binary,e_kernel,iterations=1)
                    right_binary = mp.map(cv.dilate(mp.inv(right_binary),d_kernel,iterations=1))

                    # apply AND etween left and right
                    ZDFDict[k] = cv.bitwise_and(mp.inv(left_binary), mp.inv(right_binary), mask=roi)

                # get pixels from ZDF shift with max match
                pixel_shift = util.getMaxZDF(ZDFDict)
                # convert pixel to angle
                ZDFangle = servo.getAngleNormalised(right.intrinsic, pixel_shift)

                # CENTROID CALCULATION

                centroid_l = centroid_r = md.centroidCalc(ZDFDict[pixel_shift]) # get centroid from ZDF - the same centroid for both left and right, because ZDF is their shared edges
                marked = cv.circle(ZDFDict[pixel_shift],centroid_l,5,255,-1)

                n_angles = angles
                if centroid_l != (None, None) and centroid_r != (None, None):
                    rel_ang = servo.getAngles(centroid_l, centroid_r, left.intrinsic, right.intrinsic)
                    print("rel_ang",rel_ang)
                    n_angles=util.updateAngles(angles,rel_ang)

                # Add virtual horopter angle to correct actual horopter
                n_angles = util.updateAngles(n_angles,[0,0,ZDFangle,0])

                angles = util.angleSmoothing(angles,n_angles)
                # DEBUG: IGNORE VERTICAL ROTATIONS
                # angles[1]=angles[3]=0                

                if pixel_shift!=0 or sum(rel_ang)!=0: # reset still timer
                    # print("Timer reset")
                    still_start = time.time()

                for i in range(len(servos)):
                    servos[i].write((angles[i] + offsets[i])*scaling) # move servos

                # print target position and log to file
                position = util.getPosition(angles)
                print(position)
                log.write(str(position)+"\t"+str(time.time()-starttime)+"\n")

                # reset rel_ang
                rel_ang=list()            

                curr_frame_l = cv.circle(curr_frame_l,centroid_l,5,255,-1)
                curr_frame_r = cv.circle(curr_frame_r,centroid_r,5,255,-1)

                combined = np.hstack((mp.inv(l),mp.inv(r),marked))
                rgb_frame = cv.cvtColor(combined, cv.COLOR_GRAY2RGB)                
                vlog.write(rgb_frame)
                vlog.write(rgb_frame)
                vlog.write(rgb_frame)
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
    log.write("#lost "+str(lost_counter))
    log.close()

vlog.release()
