import cv2

# Load image
img = cv2.imread('/home/tin/Pictures/Webcam/room.jpg')

# Convert to grayscale
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Apply Canny edge detection
edges = cv2.Canny(gray, 50, 150)

# Apply Sobel filter to the Canny output
sobelx = cv2.Sobel(edges, cv2.CV_64F, 1, 0, ksize=5)
abs_sobelx = cv2.convertScaleAbs(sobelx)

# Display the result
cv2.imshow('Canny + Sobel', edges)
cv2.waitKey(0)
cv2.destroyAllWindows()
