
% DoF of manipulator-------------------------------------------------------
n=3; %number of joint
p=2; %Position x-y of the E-E robot
% R and Q------------------------------------------------------------------
Q=diag([1,1,0.5,0.5]); %for the LQR
R=eye(n); % for the LQR
% Characteristics of the robot---------------------------------------------
a1=2; %length of the link #1 [m]
a2=2; %length of the link #2 [m]
a3=2; %length of the link #3 [m]
ac1=a1/2; %CoM link #1 [m]
ac2=a2/2; %CoM link #2 [m]
ac3=a3/2; %CoM link #3 [m]
m1=2; %mass link #1 [kg]
m2=2; %mass link #2 [kg]
m3=2; %mass link #3 [kg]
mb=5; %mass link #1 [kg]
mp=1; %mass EE [kg]
Izz1=1/12*m1*a1^2; %moment of inertia link #1 [kg*m^2]
Izz2=1/12*m2*a2^2; %moment of inertia link #2 [kg*m^2]
Izz3=1/12*m3*a3^2; %moment of inertia link #3 [kg*m^2]
W1=1; % (the base is modeled as a rectangular) side #1 [m]
W2=0.5; % (the base is modeled as a rectangular) side #2 [m]
Izzb=1/12*mb*(4*W1^2+4*W2^2); % moment of inertia base [kg*m^2]
ex=W1-0.1; %where the robot is connected to the base (x-direction)
ey=W2-0.1; %where the robot is connected to the base (y-direction)

mo=5; %mass target [kg]
Wo1=0.5; % (the target is modeled as a box) side #1 [m]
Wo2=0.5; % (the target is modeled as a box) side #1 [m]
Izzo=1/12*mo*(4*Wo1^2+4*Wo2^2); %moment of inertia target [kg*m^2]


% Actuator limits----------------------------------------------------------
ustall=[14;14;14];
omeganl=[0.54;0.54;0.54];
