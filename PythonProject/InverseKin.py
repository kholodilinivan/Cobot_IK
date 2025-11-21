import math
import numpy as np

def calculate_ik(ox,oy,oz):
    L1 = 134.65
    L2 = 110
    L3 = 96
    L4 = 75.05
    L5 = 63.4
    L6 = 45.2

    xc = ox
    yc = oy
    zc = oz + L6

    # calculate thetha1
    r = math.sqrt(xc**2 + yc**2)
    fi = math.asin(L5 / r) * (180 / math.pi)
    fi1 = math.atan2(xc , yc) * (180 / math.pi)
    thetha1 = fi1 - fi
    thetha1 = 90 - thetha1

    # calculate thetha2 and thetha3
    s = zc - L1
    f = math.sqrt(r**2 - L5**2)
    c = f - L4
    e = math.sqrt(s**2 + c**2)
    alpha = math.atan2(s , c) * (180 / math.pi)
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
    beta2 = math.atan2(w , L4) * (180 / math.pi)

    thetha4 = gama2 - beta2
    thetha4 = thetha4 + 90

    # calculate thetha5 and thetha6
    thetha5 = 90

    thetha6 = 90
    
    return {(thetha1,thetha2,thetha3,thetha4,thetha5,thetha6)}

# FK - to check that IK calculations are correct
import math
import numpy as np

def calculate_fk(thethas):
    L1 = 134.65
    L2 = 110
    L3 = 96
    L4 = 75.05
    L5 = 63.4
    L6 = 45.2
    
    # Extract theta values from the set
    theta_tuple = list(thethas)[0]
    thetha1 = theta_tuple[0]
    thetha2 = theta_tuple[1]
    thetha3 = theta_tuple[2]
    thetha4 = theta_tuple[3]
    thetha5 = theta_tuple[4]
    thetha6 = theta_tuple[5]
    
    alpha1 = 90; alpha2 = 0; alpha3 = 0; alpha4 = 90; alpha5 = 270; alpha6 = 0;
    r1 = 0; r2 = L2; r3 = L3; r4 = 0; r5 = 0; r6 = 0; 
    d1 = L1; d2 = 0; d3 = 0; d4 = L5; d5 = L4; d6 = L6;

    # Define cosd and sind functions (like MATLAB)
    def cosd(angle_deg):
        return math.cos(math.radians(angle_deg))
    
    def sind(angle_deg):
        return math.sin(math.radians(angle_deg))

    T1 = np.array([[cosd(thetha1), -sind(thetha1)*cosd(alpha1), sind(thetha1)*sind(alpha1), r1*cosd(thetha1)],
          [sind(thetha1), cosd(thetha1)*cosd(alpha1), -cosd(thetha1)*sind(alpha1), r1*sind(thetha1)],
          [0, sind(alpha1), cosd(alpha1), d1],
          [0, 0, 0, 1]])

    T2 = np.array([[cosd(thetha2), -sind(thetha2)*cosd(alpha2), sind(thetha2)*sind(alpha2), r2*cosd(thetha2)],
          [sind(thetha2), cosd(thetha2)*cosd(alpha2), -cosd(thetha2)*sind(alpha2), r2*sind(thetha2)],
          [0, sind(alpha2), cosd(alpha2), d2],
          [0, 0, 0, 1]])

    T3 = np.array([[cosd(thetha3), -sind(thetha3)*cosd(alpha3), sind(thetha3)*sind(alpha3), r3*cosd(thetha3)],
          [sind(thetha3), cosd(thetha3)*cosd(alpha3), -cosd(thetha3)*sind(alpha3), r3*sind(thetha3)],
          [0, sind(alpha3), cosd(alpha3), d3],
          [0, 0, 0, 1]])

    T4 = np.array([[cosd(thetha4), -sind(thetha4)*cosd(alpha4), sind(thetha4)*sind(alpha4), r4*cosd(thetha4)],
          [sind(thetha4), cosd(thetha4)*cosd(alpha4), -cosd(thetha4)*sind(alpha4), r4*sind(thetha4)],
          [0, sind(alpha4), cosd(alpha4), d4],
          [0, 0, 0, 1]])

    T5 = np.array([[cosd(thetha5), -sind(thetha5)*cosd(alpha5), sind(thetha5)*sind(alpha5), r5*cosd(thetha5)],
          [sind(thetha5), cosd(thetha5)*cosd(alpha5), -cosd(thetha5)*sind(alpha5), r5*sind(thetha5)],
          [0, sind(alpha5), cosd(alpha5), d5],
          [0, 0, 0, 1]])

    T6 = np.array([[cosd(thetha6), -sind(thetha6)*cosd(alpha6), sind(thetha6)*sind(alpha6), r6*cosd(thetha6)],
          [sind(thetha6), cosd(thetha6)*cosd(alpha6), -cosd(thetha6)*sind(alpha6), r6*sind(thetha6)],
          [0, sind(alpha6), cosd(alpha6), d6],
          [0, 0, 0, 1]])

    TT = T1 @ T2 @ T3 @ T4 @ T5 @ T6
    
    return TT

ox = 202
oy = -141
oz = 87+16
thethas = calculate_ik(ox,oy,oz)

theta_tuple = list(thethas)[0]  # Convert set to list and get first element
theta1 = theta_tuple[0]
theta2 = theta_tuple[1]
theta3 = theta_tuple[2]
theta4 = theta_tuple[3]
theta5 = theta_tuple[4]
theta6 = theta_tuple[5]

print('theta1 = ', theta1)
print('theta2 = ', theta2)
print('theta3 = ', theta3)
print('theta4 = ', theta4)
print('theta5 = ', theta5)
print('theta6 = ', theta6)

# FK - to check that IK calculations are correct
TT = calculate_fk(thethas)
print(TT)