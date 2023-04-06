import numpy as np

# offsets and constants definition
scaling = 180/270
offset = 135
offsets = [x+offset for x in [2, 7, -4, -4]]

def getAngles(cent_L, cent_R, CM_L, CM_R):
    cL = (CM_L[0,2], CM_L[1,2]) # get center point for L
    cR = (CM_R[0,2], CM_R[1,2]) # get center point for R

    fxL = CM_L[0,0]
    fyL = CM_L[1,1]

    fxR = CM_R[0,0]
    fyR = CM_R[1,1]

    # calculation for left
    uL = cL[0] - cent_L[0]
    vL = (cL[1] - cent_L[1])  # minus because of servo orientation -> negative vals for up, positive for down

    alphaHL = np.rad2deg(np.arctan2(uL, fxL))
    alphaVL = np.rad2deg(np.arctan2(vL, fyL))

    # calculation for right
    uR = cR[0] - cent_R[0]
    vR = -(cR[1] - cent_R[1])  # servo orientation here is fine. negative for down, positive for up

    alphaHR = np.rad2deg(np.arctan2(uR, fxR))
    alphaVR = np.rad2deg(np.arctan2(vR, fyR))

    # equalise vertical angles for both cameras to ensure center projection lines are coplanar
    alphaV = (abs(alphaVL) + abs(alphaVR)) / 2

    alphaVL = -alphaV if alphaVL<0 else alphaV
    alphaVR = -alphaV if alphaVR<0 else alphaV

    return [alphaHL, alphaVL, alphaHR, alphaVR]