
%This script can be used to simulate a robotic arm on a spacecraft that has
%to follow a pre-planned trajectory. The system is controlled via PD
%controler
%Michele Ambrosino 07/03/2024

clc,close all, clear all

load('param.mat')

x0 = zeros(6,1); %initial position

th_d = deg2rad(10); %desired angular base value
r_d = [0.5 0.5]'; %desired x-y displacement base
qm_d = [5;5;5]*pi/180; %desired robot koints configuration 

%Control Gain
Kp = 10*diag([100;100;1e3;1e3;1e3;1e3]); %Proportional 
Kd = diag([5;5;50;50;50;50]); %Derivative


% In this example, the trajectory is designed in different steps
Ts = 1e-2; %samplig time trajectory 
tt1 = [0:Ts:5]; %Fist duration
ind = size(tt1,2);
tt = [0:Ts:15]; %Second duration
ind2 = size(tt,2);

%Trajectory planning wiht polynomial interpolation

thd = zeros(ind2,1);
rd = zeros(ind2,size(r_d,1));
qd = zeros(ind2,size(qm_d,1));
[th,~,~] = jtraj(x0(1),th_d,tt1); %%% desired trajectory for the angular position of the base
thd(1:ind,1) = th; thd(ind+1:end,1) = th(end,1);
[r,~,~] = jtraj(x0(1:2),r_d,tt1); %%% desired trajectory for the x-y displacements of the base
rd(ind-1:2*(ind-1),:) = r; rd(2*(ind-1)+1:end,:) = ones(ind,1)*r(end,:);
[q,~,~] = jtraj(x0(4:6),qm_d,tt1); %%% desired trajectory fot the robot joints
qd(1:2*ind-2,:) = ones(2*ind-2,1)*q(1,:) ; qd(2*ind-1:end,:) = q;


xc = [x0;x0]; %%initial position and velocity for simulations
X_sol = []; %% vector to store the results

%for simulations

tic
for i = 1:size(tt,2)
    
    i
    %control PD function 
    tau = PD_control_scheme(xc,[rd(i,:)';thd(i,:);qd(i,:)'],Kp,Kd);
    
    %robot simulations
    [~,x] = ode15s(@(t,y) FFS_dynamic_model(t,y,tau,par),[0 Ts],xc);
    
    %update initial condition
    xc = x(end,:)';
    %store results
    X_sol = [X_sol;xc'];
end
toc

%plot results
figure(1)
plot(tt,X_sol(:,3),tt,thd);xlabel('time[s]');ylabel('\theta')

figure(2)
plot(tt,X_sol(:,1:2),tt,rd);xlabel('time[s]');ylabel('x-y')

figure(3)
plot(tt,X_sol(:,4:6),tt,qd);xlabel('time[s]');ylabel('qr1-qr2')


print_system_config



function tau = PD_control_scheme(x,ref,Kp,Kd)


tau(1:2,1) = Kp(1:2,1:2)*(ref(1:2)-x(1:2))-Kd(1:2,1:2)*x(7:8); %PD for the angular position of the base

tau(3,1) = Kp(3,3)*(ref(3)-x(3))-Kd(3,3)*x(9); %PD for the angular position of the base


tau(4:6,1) = Kp(4:6,4:6)*(ref(4:6)-x(4:6))-Kd(4:6,4:6)*x(10:12); %PD for the angular position of the base


end
