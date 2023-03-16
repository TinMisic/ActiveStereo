import cv2 as cv

def centroidOfDif(frame1, frame2, mov_thresh):
    difference = cv.absdiff(frame1, frame2)
    threshold = cv.threshold(difference, 25, 255, cv.THRESH_BINARY)[1]

    centroid = (None, None)    
    if threshold.sum() > mov_thresh:
        M = cv.moments(threshold)

        cX = int(M['m10'] / M['m00'])
        cY = int(M['m01'] / M['m00'])
        centroid = (cX, cY)

    return threshold, centroid

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
    cap = cv.VideoCapture(0)

    ret, start_frame = cap.read()
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        cap.release()
        exit()

    start_frame = cv.cvtColor(start_frame, cv.COLOR_RGB2GRAY)
    start_frame = cv.GaussianBlur(start_frame, (5, 5), 0)
    
    old_centorid = (None, None)
    while True:
        ret, new_frame = cap.read()
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            cap.release()
            exit()

        new_frame = cv.cvtColor(new_frame, cv.COLOR_RGB2GRAY)
        new_frame = cv.GaussianBlur(new_frame, (5, 5), 0)

        threshold, centroid = centroidOfDif(start_frame, new_frame, 10000)
        img = cv.cvtColor(threshold, cv.COLOR_GRAY2RGB)
        centroid = centroidSmoothing(old_centorid, centroid, 0.85)
        print(centroid)
        if centroid != (None, None):
            cv.circle(img, centroid, 10, color=(0, 0, 255), thickness=-1)
            cv.putText(img, "target", (centroid[0] - 25, centroid[1] - 25),cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        
        start_frame = new_frame
        old_centorid = centroid
        cv.imshow("motion",img)
        if cv.waitKey(1) == ord('q'):
            cv.imwrite("diff.jpg",threshold)
            break
    
    cap.release()
    cv.destroyAllWindows()

