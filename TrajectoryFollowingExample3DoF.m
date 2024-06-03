
%This script can be used to simulate a 3Dof robotic arm on a spacecraft. 
%Trajectory: pre-planned polynomial trajectory. 
%Controller: The system is controlled via PD controllers.

%Michele Ambrosino 06/03/2024

% To be sure that our environment is clean before starting the code
clearvars
clc,close all  

load('param.mat') %load system dynamic paramters

%System Description:
% the robotic system has 6 position (x,y,psi) for the chaser and (q1,q2,q3) robot joints. 

%% System Settings + Control Design
nx = 6; %dimension of the system state (position)
x0 = zeros(nx,1); %vector for initial position
dx0 = zeros(nx,1); %vector for initial velocity
xc = [x0;dx0]; %%initial position and velocity for simulations
X_sol = []; %% vector to store the results
U_sol =[]; %vector to store the input

%Control Gain
Kp = 10*diag([100;100;1e3;1e3;1e3;1e3]); %Proportional 
Kd = diag([5;5;50;50;50;50]); %Derivative

%% Trajectory Planning Definition
th_d = deg2rad(10); %desired angular base value [rad]
r_d = [0.5 0.5]'; %desired x-y displacement base [m]
qm_d = deg2rad([5;5;5]); %desired robot koints configuration [rad];
Ts = 1e-2; %samplig time trajectory [s]
tEnd = 5; %duration of each sub-trajectory [s]
tEnd_f = 3*tEnd; %Concatenation of the three trajectory [s]

% In this example, the trajectory is designed in different steps
sub_time_vector = [0:Ts:tEnd]; %Each sub-trajectory is planned for tEnd seconds
ind = size(sub_time_vector,2); %dimension of sub-trajectory
tSpan = [0:Ts:tEnd_f]; %The full trajectory is tEnd_f [s]
ind2 = size(tSpan,2); %dimension of full-trajectory

%Here we build the three desidered trajectory
%First we move theta for tEnd then we keep it constant for the rest
%Second we move the base x-y from tEnd to 2*tEnd
%Third we move the robotic arm from 2*tEnd tp tEnd_f
thd = zeros(ind2,1); 
rd = zeros(ind2,size(r_d,1));
qd = zeros(ind2,size(qm_d,1));

%theta traejctory  (the poly_traj is the same function that is present in
%the Robotic Toolboox by P.Corke, to avoid to install the toolbox you can
%use the custom poly_traj.m
[th,~,~] = poly_traj(x0(1),th_d,sub_time_vector); %%% desired trajectory for the angular position of the base
thd(1:ind,1) = th; thd(ind+1:end,1) = th(end,1);
%x-y trajectory
[r,~,~] = poly_traj(x0(1:2),r_d,sub_time_vector); %%% desired trajectory for the x-y displacements of the base
rd(ind-1:2*(ind-1),:) = r; rd(2*(ind-1)+1:end,:) = ones(ind,1)*r(end,:);
%robot joint trajectory
[q,~,~] = poly_traj(x0(4:6),qm_d,sub_time_vector); %%% desired trajectory fot the robot joints
qd(1:2*ind-2,:) = ones(2*ind-2,1)*q(1,:) ; qd(2*ind-1:end,:) = q;



%% Simulation start 
for i = 1:ind2
    
    disp("Iteration: "+i+" / "+ind2)
    %control PD function 
    tau = PD_control_scheme(xc,[rd(i,:)';thd(i,:);qd(i,:)'],Kp,Kd);
    
    %robot simulations
    [~,x] = ode15s(@(t,y) FFS_dynamic_model(t,y,tau,par),[0 Ts],xc);
    
    %update initial condition
    xc = x(end,:)';
    %store results
    U_sol = [U_sol;tau'];
    X_sol = [X_sol;xc'];
end

%% Print Plot Results
printRes = false;

if printRes
    lW = 2.5; %LineWidth
    fS = 18;  %FontSize
    figure(1)
    subplot(3,1,1); plot(tSpan,X_sol(:,3),tSpan,thd,'LineWidth',lW); legend('\theta','\theta_{r}','Orientation','horizontal','Location','best')
    title('position states')
    ylabel('\theta'); grid on; set(gca,'fontsize', fS)
    
    subplot(3,1,2); plot(tSpan,X_sol(:,1:2),tSpan,rd,'LineWidth',lW);legend('x','y','x_{r}','y_{r}','Orientation','horizontal','Location','best')
    ylabel('x-y'); grid on; set(gca,'fontsize', fS)
    
    subplot(3,1,3); plot(tSpan,X_sol(:,4:6),tSpan,qd,'LineWidth',lW); legend('q_1','q_2','q_2','q_{r1}','q_{r2}','q_{r3}','Orientation','horizontal','Location','best')
    xlabel('time[s]');ylabel('q1-q2'); grid on; set(gca,'fontsize', fS)
    
    figure(2)
    subplot(3,1,1); plot(tSpan,U_sol(:,3),'LineWidth',lW); 
    title('control input')
    ylabel('u_{\theta}'); grid on; set(gca,'fontsize', fS)
    
    subplot(3,1,2); plot(tSpan,U_sol(:,1:2),'LineWidth',lW);
    ylabel('u_{x-y}'); grid on; set(gca,'fontsize', fS)
    
    subplot(3,1,3); plot(tSpan,U_sol(:,4:6),'LineWidth',lW); 
    xlabel('time[s]');ylabel('u_{q1-q2}'); grid on; set(gca,'fontsize', fS)
  
    
end

%% Print Animation
printAnimation = true;

if printAnimation
    
    print_system_config
    
end




%this is the PD function
%input: x:state system [pos;vel]; ref:reference trajectory; 
%Kp:proportional gain; Kd: damping gain
%output: tau: torque vector
function tau = PD_control_scheme(x,ref,Kp,Kd)

tau(1:2,1) = Kp(1:2,1:2)*(ref(1:2)-x(1:2))-Kd(1:2,1:2)*x(7:8); %PD for the angular position of the base
tau(3,1) = Kp(3,3)*(ref(3)-x(3))-Kd(3,3)*x(9); %PD for the angular position of the base
tau(4:6,1) = Kp(4:6,4:6)*(ref(4:6)-x(4:6))-Kd(4:6,4:6)*x(10:12); %PD for the angular position of the base


end
