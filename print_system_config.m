

% Characteristics of the robot---------------------------------------------
a1= par.a1; a2 = par.a2; a3 = par.a3;    
ac1 = par.ac1; ac2 = par.ac2; ac3 = par.ac3;
m1 = par.m1; m2 = par.m2; m3 = par.m3;
mb = par.mb; mp = par.mp;
Izz1 = par.Izz1; Izz2 = par.Izz2; Izz3 = par.Izz3; 
W1 = par.W1; W2 = par.W2; 
Izzb = par.Izzb; 
ex = par.ex; ey = par.ey;


x = X_sol'; %take the X_sol of the main script to do some manipulation

%Un-pack the results
xc=x(1,:); %x_base
yc=x(2,:); %y_base
psi=x(3,:); %orient_base
q1=x(4,:); %q1 robot
q2=x(5,:); %q2 robot
q3=x(6,:); %q3 robot


%From here there this the direct kinematics of the robot to have the x-y of
%the robotic arm 

%x-y of the base
xb=xc+ex*cos(psi)+ey*sin(psi); 
yb=yc-ey*cos(psi)+ex*sin(psi); 

%x-y of the first link of the robotic arm
xe1=xc+a1*cos(psi+q1)+ex*cos(psi)+ey*sin(psi); 
ye1=yc+a1*sin(psi+q1)-ey*cos(psi)+ex*sin(psi);

%x-y of the second link of the robotic arm
xe2=xc+a1*cos(psi+q1)+ex*cos(psi)+ey*sin(psi)+a2*cos(psi+q1+q2);
ye2=yc+a1*sin(psi+q1)-ey*cos(psi)+ex*sin(psi)+a2*sin(psi+q1+q2);

%x-y of the third (end-effector) link of the robotic arm
xe3=xc+a3*cos(psi+q1+q2+q3)+a1*cos(psi+q1)+ex*cos(psi)+ey*sin(psi)+a2*cos(psi+q1+q2);
ye3=yc+a3*sin(psi+q1+q2+q3)+a1*sin(psi+q1)-ey*cos(psi)+ex*sin(psi)+a2*sin(psi+q1+q2);


figure()

%some plotting setting
hold on;grid on;box on
axis equal
lW = 3; c1 = [0,0,1]; c2 = [1,0,0];

printMotion = false; %true if you want to have the motion

for i = 1:10:size(x,2)

x11=xc(i)+W1.*cos(psi(i))-W2.*sin(psi(i));
y11=yc(i)+W1.*sin(psi(i))+W2.*sin(psi(i)+pi/2);
x21=xc(i)-W1.*cos(psi(i))+W2.*cos(psi(i)+pi/2);
y21=yc(i)-W1.*sin(psi(i))+W2.*sin(psi(i)+pi/2);
x31=xc(i)+W1.*cos(psi(i))+W2.*sin(psi(i));
y31=yc(i)+W1.*sin(psi(i))-W2.*sin(psi(i)+pi/2);
x41=xc(i)-W1.*cos(psi(i))-W2.*cos(psi(i)+pi/2);
y41=yc(i)-W1.*sin(psi(i))-W2.*sin(psi(i)+pi/2);
Line1 = line([xb(i);xe1(i)],[yb(i);ye1(i)],'color',c1,'LineWidth',lW);
Line2 = line([xe1(i);xe2(i)],[ye1(i);ye2(i)],'color',c1,'LineWidth',lW);
Line3 = line([xe2(i);xe3(i)],[ye2(i);ye3(i)],'color',c1,'LineWidth',lW);
Line4 = line([x11;x31],[y11;y31],'color',c2,'LineWidth',lW);
Line5 = line([x41;x31],[y41;y31],'color',c2,'LineWidth',lW);
Line6 = line([x21;x41],[y21;y41],'color',c2,'LineWidth',lW);
Line7 = line([x21;x11],[y21;y11],'color',c2,'LineWidth',lW);


pause(0.1)

if i == size(x,2); break 
end

if printMotion == false
delete(Line1);
delete(Line2);
delete(Line3);
delete(Line4);
delete(Line5);
delete(Line6);
delete(Line7);
end



end