import math
import numpy as np


def calculate_ik(ox,oy,oz):
    L1 = 131.56
    L2 = 110.4
    L3 = 96
    L4 = 73.18
    L5 = 66.39
    L6 = 43.6
    '''
    ox = float(input('ox '))
    oy = float(input('oy '))
    oz = float(input('oz ')) 
    '''
    # ox = 245.231
    # oy = 56.149
    # oz = 139.411

    T = np.array([[-1, 0,  0, ox], 
         [0, 1,  0, oy], 
         [0, 0, -1, oz], 
         [0, 0,  0, 1]])

    # Rotation matrix of the ik point
    R = np.array([[T[0][0], T[1][0], T[2][0]],
         [T[0][1], T[1][1], T[2][1]],
         [T[0][2], T[1][2], T[2][2]]])

    o = [ox, oy, oz]  # center point calculation
    xc = o[0] - L6 * R[2][0]
    yc = o[1] - L6 * R[2][1]
    zc = o[2] - L6 * R[2][2]

    # calculate thetha1
    r = math.sqrt(xc**2 + yc**2)
    fi = math.asin(L5 / r) * (180 / math.pi)
    fi1 = math.atan(xc / yc) * (180 / math.pi)
    thetha1 = fi1 - fi
    thetha1 = 90 - thetha1

    # calculate thetha2 and thetha3
    s = zc - L1
    f = math.sqrt(r**2 - L5**2)
    c = f - L4
    e = math.sqrt(s**2 + c**2)
    alpha = math.atan(s / c) * (180 / math.pi)
    D1 = (L2**2 + e**2 - L3**2) / (2 * L2 * e)
    gama = math.acos(D1) * (180 / math.pi)
    D2 = (L2**2 + L3**2 - e**2) / (2 * L2 * L3)
    gama1 = math.acos(D2) * (180 / math.pi)

    thetha2 = alpha + gama

    thetha3 = -(180 - gama1)

    # calculate thetha4
    x = f * math.tan(math.radians(alpha))
    w = x - s
    gama2 = 180 - gama - gama1
    beta = 180 - gama2
    beta2 = math.atan(w / L4) * (180 / math.pi)

    thetha4 = gama2 - beta2
    thetha4 = thetha4 + 90

    # calculate thetha5 and thetha6
    thetha5 = 90

    thetha6 = 90

    # thetha3 = -thetha3 + 90

    '''
    print('thetha1 = ', thetha1)
    print('thetha2 = ', thetha2)
    print('thetha3 = ', thetha3)
    print('thetha4 = ', thetha4)
    print('thetha5 = ', thetha5)
    print('thetha6 = ', thetha6)
    '''
    
    return {(thetha1,thetha2,thetha3,thetha4,thetha5,thetha6)}
