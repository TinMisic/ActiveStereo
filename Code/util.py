import abc
import cv2
import numpy as np

# Mapping 
class ImgMap(metaclass=abc.ABCMeta):
    '''Abstract class that represents an image mapping. \n\nHas two functions: map & inv'''

    @abc.abstractmethod
    def map(self, input_img):
        '''Transform the image'''
        pass

    @abc.abstractmethod
    def inv(self, input_img):
        '''Inverse of map'''
        pass

class IdentityMap(ImgMap):
    '''Identity mapping: apply no transformation'''

    def map(self, input_img):
        return input_img
    
    def inv(self, input_img):
        return input_img
    
class LogPolarMap(ImgMap):
    '''Log Polar mapping'''

    def __init__(self, o_shape, lpm_shape) -> None:
        self.o_shape = o_shape
        self.lpm_shape = lpm_shape
        self.center = (o_shape[0]//2, o_shape[1]//2)

    def map(self, input_img):
        return cv2.warpPolar(input_img, self.lpm_shape, self.center, self.o_shape[0]*0.90*0.5,cv2.WARP_POLAR_LOG + cv2.WARP_FILL_OUTLIERS)
    
    def inv(self, input_img):
        return cv2.warpPolar(input_img, self.o_shape, self.center, self.o_shape[0]*0.90*0.5,cv2.WARP_POLAR_LOG + cv2.WARP_INVERSE_MAP +cv2.WARP_FILL_OUTLIERS)
    
# Edge detection
class EdgeDetector(metaclass=abc.ABCMeta):
    '''Abstract edge detector class. \n\nOnly has one function: detect'''

    @abc.abstractmethod
    def detect(self, input_img):
        '''Detect edges'''
        pass

class SobelCartesian(EdgeDetector):
    '''Sobel vertical edge detector'''
    
    def __init__(self, ksize=7):
        self.ksize = ksize

    def detect(self, input_img):
        # Apply Sobel filter to get vertical edges
        sobelx = cv2.Sobel(input_img, cv2.CV_64F, 1, 0, self.ksize)

        # Convert the output back to uint8 and normalize
        abs_sobelx = np.absolute(sobelx)
        scaled_sobel = np.uint8(255*abs_sobelx/np.max(abs_sobelx))
        return scaled_sobel
    
class SobelLPM(EdgeDetector):
    '''Sobel vertical edge detector on LPM image. NOT IMPLEMENTED'''
    def detect(self, input_img):
        pass

class SobelPseudoLPM(EdgeDetector):
    '''Sobel vertical edge detector on LPM image. 
    
    This detector does not work directly on the LPM. 
    
    The procedure is: convertToCartesian(image)->cartesianSobel(image)->convertToLPM(image)'''
    def __init__(self, mapping: ImgMap, ksize = 7):
        self.mapping = mapping
        self.ksize = ksize

    def detect(self, input_img):
        # convert to cartesian
        cart = self.mapping.inv(input_img)

        # Apply Sobel filter to get vertical edges
        sobelx = cv2.Sobel(cart, cv2.CV_64F, 1, 0, self.ksize)

        # Convert the output back to uint8 and normalize
        abs_sobelx = np.absolute(sobelx)
        scaled_sobel = np.uint8(255*abs_sobelx/np.max(abs_sobelx))

        # dilate edges
        kernel = np.ones((3,3),np.uint8)

        # Apply dilation to the image
        dilated = cv2.dilate(scaled_sobel, kernel, iterations = 1)

        # convert back to LPM
        detected = self.mapping.map(dilated)

        return detected


# Shift operator
class ShiftOp(metaclass=abc.ABCMeta):
    '''Abstract class for a horizontal shift operation. Has only one function: shift'''

    @abc.abstractmethod
    def shift(self, input_img, delta):
        """Shifts the image horizontaly by delta units"""
        pass

class ShiftCartesian(ShiftOp):
    '''Horizontal shift using cartesian coordinates'''

    def shift(self, input_img, delta):
        M = np.float32([[1, 0, delta],[0, 1, 0]])
        return cv2.warpAffine(input_img, M, (input_img.shape[1], input_img.shape[0]))
    
class ShiftLPM(ShiftOp):
    '''Horizontal shift using log-polar coordinates. NOT IMPLEMENTED'''

    def shift(self, input_img, delta):
        pass

class ShiftPseudoLPM(ShiftOp):
    '''Horizontal shift using LPM. 
    
    This operator does not work directly on the LPM. 
    
    The procedure is: convertToCartesian(image)->shift(image)->convertToLPM(image)'''

    def __init__(self, mapping: ImgMap):
        self.mapping = mapping

    def shift(self, input_img, delta):
        # convert image to cartesian
        cart = self.mapping.inv(input_img)

        # shift the image by delta
        M = np.float32([[1, 0, delta],[0, 1, 0]])
        shft = cv2.warpAffine(input_img, M, (input_img.shape[1], input_img.shape[0]))

        # convert back to lpm
        final = self.mapping.map(shft)

        return final
    

def generateShifts(image, shifter:ShiftOp, shifts=11, stride=4):
    """Generates the appropriate shifts for small rotations of the camera"""
    units = [stride*x for x in range(-(shifts//2),shifts//2+1)]
    res = dict()
    for unit in units:
        res[unit] = shifter.shift(image,unit)

    return res

def getMaxZDF(ZDFDict):
    """Takes dictionary of ZDF images and returns the one with the maximum sum"""
    maxval = 0
    maxunit = 0
    for k in ZDFDict.keys():
        val = ZDFDict[k].sum()
        if val>maxval:
            maxval = val
            maxunit = k

    return maxunit

def updateAngles(alphas, rel_ang, limit=90):
    summ = [alphas[i] + rel_ang[i] for i in range(len(alphas))]
    limited = []
    for s in summ:
        if s>limit:
            limited.append(limit)
        elif s<(-limit):
            limited.append(-limit)
        else:
            limited.append(s)
    
    return limited

def angleSmoothing(old, new, factor=0.85):
    return [old[i]*factor + new[i]*(1-factor) for i in range(len(old))]

if __name__=="__main__":
    # Load image
    img = cv2.imread('/home/tin/Downloads/marcus.png')

    # Convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cv2.imshow("Original Image",gray)

    # Initialize classes
    mp = LogPolarMap((gray.shape[1],gray.shape[0]),(100,100))
    edgeDetect = SobelPseudoLPM(mp)
    shift = ShiftPseudoLPM(mp)

    m = mp.map(gray)
    cv2.imshow("Mapping result",m)
    cv2.imwrite("realLPM.png",m)

    e = edgeDetect.detect(m)
    cv2.imshow("Detected edges:",e)

    i = mp.inv(m)
    cv2.imshow("Inverted mapping",i)

    merged = np.hstack((gray,i))
    cv2.imwrite("lpm.png",merged)

    s = shift.shift(i, -20)
    cv2.imshow("Shift",s)

    cv2.waitKey()
    cv2.destroyAllWindows()
    print(gray.shape)
