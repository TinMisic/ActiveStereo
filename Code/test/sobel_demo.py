import cv2
import numpy as np

# Load image
img = cv2.imread('/home/tin/Pictures/Webcam/room.jpg')

# Convert to grayscale
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Apply Sobel filter to get vertical edges
sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=7)

# Convert the output back to uint8 and normalize
abs_sobelx = np.absolute(sobelx)
scaled_sobel = np.uint8(255*abs_sobelx/np.max(abs_sobelx))

# Display the result
cv2.imshow('Sobel Vertical Edge Detection', scaled_sobel)
cv2.waitKey(0)
cv2.destroyAllWindows()
