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
        sobelx = cv2.Sobel(input_img, cv2.CV_64F, 0, 1, self.ksize)

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
        sobelx = cv2.Sobel(cart, cv2.CV_64F, 0, 1, self.ksize)

        # Convert the output back to uint8 and normalize
        abs_sobelx = np.absolute(sobelx)
        scaled_sobel = np.uint8(255*abs_sobelx/np.max(abs_sobelx))

        # convert back to LPM
        detected = self.mapping.map(scaled_sobel)

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

if __name__=="__main__":
    # Load image
    img = cv2.imread('/home/tin/Pictures/Webcam/room.jpg')

    # Convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cv2.imshow("Original Image",gray)

    # Initialize classes
    mp = LogPolarMap((gray.shape[1],gray.shape[0]),(100,100))
    edgeDetect = SobelPseudoLPM(mp)
    shift = ShiftPseudoLPM(mp)

    m = mp.map(gray)
    cv2.imshow("Mapping result",m)

    e = edgeDetect.detect(m)
    cv2.imshow("Detected edges:",e)

    i = mp.inv(e)
    cv2.imshow("Inverted mapping",i)

    s = shift.shift(i, -20)
    cv2.imshow("Shift",s)

    cv2.waitKey()
    cv2.destroyAllWindows()