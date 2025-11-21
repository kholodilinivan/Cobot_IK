%% Kinematic Diagram Config 1
L1 = 131.56; L2 = 110.4; L3 = 96; L4 = 73.18; L5 = 66.39; L6 = 43.6;
alpha1 = 90; alpha2 = 0; alpha3 = 0; alpha4 = 90; alpha5 = 270; alpha6 = 0;
thetha1 = 90; thetha2 = 90; thetha3 = 0; thetha4 = 90; thetha5 = 0; thetha6 = 0;
r1 = 0; r2 = L2; r3 = L3; r4 = 0; r5 = 0; r6 = 0; 
d1 = L1; d2 = 0; d3 = 0; d4 = L5; d5 = L4; d6 = L6;

L(1) = Revolute('d',d1,'a',r1,'alpha',alpha1*pi/180);
L(2) = Revolute('d',d2,'a',r2,'alpha',alpha2*pi/180);
L(3) = Revolute('d',d3,'a',r3,'alpha',alpha3*pi/180);
L(4) = Revolute('d',d4,'a',r4,'alpha',alpha4*pi/180);
L(5) = Revolute('d',d5,'a',r5,'alpha',alpha5*pi/180);
L(6) = Revolute('d',d6,'a',r6,'alpha',alpha6*pi/180);
robot = SerialLink(L);
joints = [thetha1*pi/180,thetha2*pi/180,thetha3*pi/180,thetha4*pi/180,thetha5*pi/180,thetha6*pi/180];
robot.plot(joints);
robot.teach(joints);
%% Kinematic Diagram Config 2
L1 = 131.56; L2 = 110.4; L3 = 96; L4 = 73.18; L5 = 66.39; L6 = 43.6;
alpha1 = 90; alpha2 = 0; alpha3 = 0; alpha4 = 90; alpha5 = 270; alpha6 = 0;
thetha1 = 0; thetha2 = 0; thetha3 = 0; thetha4 = 90; thetha5 = 90; thetha6 = 90;
r1 = 0; r2 = L2; r3 = L3; r4 = 0; r5 = 0; r6 = 0; 
d1 = L1; d2 = 0; d3 = 0; d4 = L5; d5 = L4; d6 = L6;

L(1) = Revolute('d',d1,'a',r1,'alpha',alpha1*pi/180);
L(2) = Revolute('d',d2,'a',r2,'alpha',alpha2*pi/180);
L(3) = Revolute('d',d3,'a',r3,'alpha',alpha3*pi/180);
L(4) = Revolute('d',d4,'a',r4,'alpha',alpha4*pi/180);
L(5) = Revolute('d',d5,'a',r5,'alpha',alpha5*pi/180);
L(6) = Revolute('d',d6,'a',r6,'alpha',alpha6*pi/180);
robot = SerialLink(L);
joints = [thetha1*pi/180,thetha2*pi/180,thetha3*pi/180,thetha4*pi/180,thetha5*pi/180,thetha6*pi/180];
robot.plot(joints);
robot.teach(joints);
%% MyCobot280 IK Simplified

clc
clear

L1 = 131.56; L2 = 110.4; L3 = 96; L4 = 73.18; L5 = 66.39; L6 = 43.6;
ox = 245.231;
oy = 56.149;
oz = 139.411;

xc = ox;
yc = oy;
zc = oz + L6;

% find thetha1
r = sqrt(xc^2+yc^2);
fi = asind(L5/r);
fi1 = atan2d(xc,yc);

thetha1 = fi1 - fi
thetha1 = 90 - thetha1

% find thetha2 and thetha3
s = zc - L1; 
% c = yc - L4;
f = sqrt(r^2-L5^2);
c = f - L4;
e = sqrt(s^2+c^2);
alpha = atan2d(s,c);
D1 = (L2^2+e^2-L3^2)/(2*L2*e);
gama = acosd(D1);
D2 = (L2^2+L3^2-e^2)/(2*L2*L3);
gama1 = acosd(D2);

thetha2 = (alpha + gama) % config II

thetha3 = -(180 - gama1) % config II
% thetha3 = thetha3 + 90

% find thetha4
x = f*tand(alpha);
w = x - s;
gama2 = 180 - gama - gama1;
beta = 180 - gama2;
beta2 = atan2d(w,L4);
thetha4 = gama2 - beta2;
thetha4 = thetha4 + 90;

% find thetha5

thetha5 = 90

% find thetha6
thetha6 = 90

alpha1 = 90; alpha2 = 0; alpha3 = 0; alpha4 = 90; alpha5 = 270; alpha6 = 0;
r1 = 0; r2 = L2; r3 = L3; r4 = 0; r5 = 0; r6 = 0; 
d1 = L1; d2 = 0; d3 = 0; d4 = L5; d5 = L4; d6 = L6;

L(1) = Revolute('d',d1,'a',r1,'alpha',alpha1*pi/180);
L(2) = Revolute('d',d2,'a',r2,'alpha',alpha2*pi/180);
L(3) = Revolute('d',d3,'a',r3,'alpha',alpha3*pi/180);
L(4) = Revolute('d',d4,'a',r4,'alpha',alpha4*pi/180);
L(5) = Revolute('d',d5,'a',r5,'alpha',alpha5*pi/180);
L(6) = Revolute('d',d6,'a',r6,'alpha',alpha6*pi/180);
robot = SerialLink(L);
joints = [thetha1*pi/180,thetha2*pi/180,thetha3*pi/180,thetha4*pi/180,thetha5*pi/180,thetha6*pi/180];
robot.plot(joints);
robot.teach(joints);
%% MyCobot280 IK

clc
clear

x=0; % Roll -90
y=180; % Pitch 0
z=0; % Yaw 0
RR = compose_rotation(x, y, z)

L1 = 131.56; L2 = 110.4; L3 = 96; L4 = 73.18; L5 = 66.39; L6 = 43.6;

T = [RR(1,1)    RR(1,2)   RR(1,3)    245.231
     RR(2,1)    RR(2,2)   RR(2,3)    56.149
     RR(3,1)    RR(3,2)   RR(3,3)    139.411
     0    0    0    1.0000];
     
R = [T(1,1)    T(1,2)   T(1,3)
     T(2,1)    T(2,2)   T(2,3)
     T(3,1)    T(3,2)   T(3,3)]; 

o = [T(1,4); T(2,4); T(3,4)];

xc = o(1) - L6*R(1,3);
yc = o(2) - L6*R(2,3);
zc = o(3) - L6*R(3,3);

% find thetha1
r = sqrt(xc^2+yc^2);
fi = asind(L5/r);
fi1 = atan2d(xc,yc);

thetha1 = fi1 - fi
thetha1 = 90 - thetha1

% find thetha2 and thetha3
s = zc - L1; 
% c = yc - L4;
f = sqrt(r^2-L5^2);
c = f - L4;
e = sqrt(s^2+c^2);
alpha = atan2d(s,c);
D1 = (L2^2+e^2-L3^2)/(2*L2*e);
gama = acosd(D1);
D2 = (L2^2+L3^2-e^2)/(2*L2*L3);
gama1 = acosd(D2);

thetha2 = (alpha + gama) % config II

thetha3 = -(180 - gama1) % config II
% thetha3 = thetha3 + 90

% find thetha4
x = f*tand(alpha);
w = x - s;
gama2 = 180 - gama - gama1;
beta = 180 - gama2;
beta2 = atan2d(w,L4);
thetha4 = gama2 - beta2;
thetha4 = thetha4 + 90;

% find thetha5

thetha5 = 90

% find thetha6
thetha6 = 90

alpha1 = 90; alpha2 = 0; alpha3 = 0; alpha4 = 90; alpha5 = 270; alpha6 = 0;
r1 = 0; r2 = L2; r3 = L3; r4 = 0; r5 = 0; r6 = 0; 
d1 = L1; d2 = 0; d3 = 0; d4 = L5; d5 = L4; d6 = L6;

L(1) = Revolute('d',d1,'a',r1,'alpha',alpha1*pi/180);
L(2) = Revolute('d',d2,'a',r2,'alpha',alpha2*pi/180);
L(3) = Revolute('d',d3,'a',r3,'alpha',alpha3*pi/180);
L(4) = Revolute('d',d4,'a',r4,'alpha',alpha4*pi/180);
L(5) = Revolute('d',d5,'a',r5,'alpha',alpha5*pi/180);
L(6) = Revolute('d',d6,'a',r6,'alpha',alpha6*pi/180);
robot = SerialLink(L);
joints = [thetha1*pi/180,thetha2*pi/180,thetha3*pi/180,thetha4*pi/180,thetha5*pi/180,thetha6*pi/180];
robot.plot(joints);
robot.teach(joints);
%%
L1 = 131.56; L2 = 110.4; L3 = 96; L4 = 73.18; L5 = 66.39; L6 = 43.6;
alpha1 = 90; alpha2 = 0; alpha3 = 0; alpha4 = 90; alpha5 = 270; alpha6 = 0;
thetha1 = 28; thetha2 = 45; thetha3 = -60; thetha4 = 103.6; thetha5 = 91.1; thetha6 = 90;
r1 = 0; r2 = L2; r3 = L3; r4 = 0; r5 = 0; r6 = 0; 
d1 = L1; d2 = 0; d3 = 0; d4 = L5; d5 = L4; d6 = L6;

L(1) = Revolute('d',d1,'a',r1,'alpha',alpha1*pi/180);
L(2) = Revolute('d',d2,'a',r2,'alpha',alpha2*pi/180);
L(3) = Revolute('d',d3,'a',r3,'alpha',alpha3*pi/180);
L(4) = Revolute('d',d4,'a',r4,'alpha',alpha4*pi/180);
L(5) = Revolute('d',d5,'a',r5,'alpha',alpha5*pi/180);
L(6) = Revolute('d',d6,'a',r6,'alpha',alpha6*pi/180);
robot = SerialLink(L);
joints = [thetha1*pi/180,thetha2*pi/180,thetha3*pi/180,thetha4*pi/180,thetha5*pi/180,thetha6*pi/180];
robot.plot(joints);
robot.teach(joints);