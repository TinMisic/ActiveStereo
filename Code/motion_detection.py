import cv2 as cv
import numpy as np

def centroidOfDif(frame1, frame2, mov_thresh=10000):
    difference = cv.absdiff(frame1, frame2)
    threshold = cv.threshold(difference, 25, 255, cv.THRESH_BINARY)[1]

    centroid = (None, None)    
    if threshold.sum() > mov_thresh:
        # print("Passed threshold",threshold.sum())
        M = cv.moments(threshold)

        cX = int(M['m10'] / M['m00'])
        cY = int(M['m01'] / M['m00'])
        centroid = (cX, cY)
        # print(centroid)

    return threshold, centroid

def centroidCalc(frame):
    cent = (None, None)
    M = cv.moments(frame)
    cX = int(M['m10'] / M['m00'])
    cY = int(M['m01'] / M['m00'])
    cent = (cX, cY)

    return cent

def centroidSmoothing(oldCent, newCent, sFactor = 0.95):
    if oldCent == (None, None):
        return newCent
    elif newCent == (None, None):
        return oldCent
    else:
        x = int(oldCent[0] * sFactor + newCent[0] * (1-sFactor))
        y = int(oldCent[1] * sFactor + newCent[1] * (1-sFactor))
        return (x, y)

if __name__=="__main__":
    cap = cv.VideoCapture(6)

    ret, start_frame = cap.read()
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        cap.release()
        exit()

    shape = (50, 50)
    margin = 0.9

    start_frame = cv.cvtColor(start_frame, cv.COLOR_RGB2GRAY)
    start_frame = cv.GaussianBlur(start_frame, (5, 5), 0)

    center = (start_frame.shape[1]//2, start_frame.shape[0]//2)
    # mask = np.zeros((start_frame.shape[0],start_frame.shape[1]), dtype=np.uint8)
    # cv.circle(mask, (start_frame.shape[1]//2,start_frame.shape[0]//2), int(start_frame.shape[1]*margin*0.5), 255, -1)

    sf_polar = cv.warpPolar(start_frame, shape, center, start_frame.shape[1]*margin*0.5, cv.WARP_POLAR_LOG + cv.WARP_FILL_OUTLIERS)
    sf_ilpm = cv.warpPolar(sf_polar, (start_frame.shape[1], start_frame.shape[0]), center, start_frame.shape[1]*margin*0.5,cv.WARP_POLAR_LOG + cv.WARP_INVERSE_MAP + cv.WARP_FILL_OUTLIERS)
    # sf_ilpm = cv.bitwise_and(sf_ilpm, sf_ilpm, mask=mask)
    # cv.imshow("sf_ilpm",sf_ilpm)
    # cv.waitKey()
    
    old_centorid = (None, None)
    while True:
        ret, new_frame = cap.read()
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            cap.release()
            exit()

        new_frame = cv.cvtColor(new_frame, cv.COLOR_RGB2GRAY)
        new_frame = cv.GaussianBlur(new_frame, (5, 5), 0)

        nf_polar = cv.warpPolar(new_frame, shape, center, new_frame.shape[1]*margin*0.5, cv.WARP_POLAR_LOG + cv.WARP_FILL_OUTLIERS)
        nf_ilpm = cv.warpPolar(nf_polar, (new_frame.shape[1], new_frame.shape[0]), center, new_frame.shape[1]*margin*0.5,cv.WARP_POLAR_LOG + cv.WARP_INVERSE_MAP + cv.WARP_FILL_OUTLIERS)
        # cv.imshow("nf_ilpm",nf_ilpm)
        # cv.waitKey()

        # threshold = np.ones((nf_ilpm.shape[0],nf_ilpm.shape[1], 1),dtype=np.uint8) * 255
        # cv.imshow("thresh_ones",threshold)
        # cv.waitKey()
        threshold, centroid = centroidOfDif(sf_ilpm, nf_ilpm, 10000)
        # cv.imshow("thresh_after centroid", threshold)
        # cv.waitKey()
        img = cv.cvtColor(threshold, cv.COLOR_GRAY2RGB)
        centroid = centroidSmoothing(old_centorid, centroid, 0.85)
        print(centroid)
        if centroid != (None, None):
            cv.circle(img, centroid, 10, color=(0, 0, 255), thickness=-1)
            cv.putText(img, "target", (centroid[0] - 25, centroid[1] - 25),cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        
        # start_frame = new_frame
        sf_ilpm = nf_ilpm
        old_centorid = centroid
        cv.imshow("motion",img)
        if cv.waitKey(1) == ord('q'):
            cv.imwrite("diff.jpg",threshold)
            break
    
    cap.release()
    cv.destroyAllWindows()

