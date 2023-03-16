# Tests if the centroid in the regular image is the same as the one calculated in the LPM and then transformed back into normal coordinates
import cv2 as cv

original = cv.imread('diff.jpg')
original = cv.cvtColor(original, cv.COLOR_RGB2GRAY)

# calculate centorid in original
M = cv.moments(original)

cX = int(M['m10'] / M['m00'])
cY = int(M['m01'] / M['m00'])
centroid = (cX, cY)

colored1 = cv.cvtColor(original, cv.COLOR_GRAY2RGB)
cv.circle(colored1, centroid, 5, (255,0,255),2)
cv.imshow("original with centroid",colored1)

# transform into LPM
shape = (128, 256)
margin = 0.90
center = (original.shape[0]/2, original.shape[1]/2)
polar = cv.warpPolar(original, shape, center, original.shape[1]*margin*0.5, cv.WARP_POLAR_LOG)
# calculate centorid in polar
M = cv.moments(polar)

cX = int(M['m10'] / M['m00'])
cY = int(M['m01'] / M['m00'])
centroid = (cX, cY)

colored2 = cv.cvtColor(polar, cv.COLOR_GRAY2RGB)
cv.circle(colored2, centroid, 5, (255,0,255),2)
cv.imshow("polar with centroid",colored2)

# recovered LPM
recovered_c = cv.warpPolar(colored2, (original.shape[1], original.shape[0]), center, original.shape[1]*margin*0.5,cv.WARP_POLAR_LOG + cv.WARP_INVERSE_MAP)
cv.imshow("recovered polar with centroid from polar",recovered_c)


recovered = cv.warpPolar(polar, (original.shape[1], original.shape[0]), center, original.shape[1]*margin*0.5,cv.WARP_POLAR_LOG + cv.WARP_INVERSE_MAP)
# calculate centorid in polar
M = cv.moments(recovered)

cX = int(M['m10'] / M['m00'])
cY = int(M['m01'] / M['m00'])
centroid = (cX, cY)

colored3 = cv.cvtColor(recovered, cv.COLOR_GRAY2RGB)
cv.circle(colored3, centroid, 5, (255,0,255),2)
cv.imshow("recovered with centorid from recovered",colored3)

cv.waitKey(0)    
cv.destroyAllWindows()

# Conclusion: Centorid calculated in LPM does not correspond to the centroid in the original/recovered image
