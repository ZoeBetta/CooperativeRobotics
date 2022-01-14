
addpath('./simulation_scripts');
clc;
clear;
close all

% Simulation variables (integration and final time)
deltat = 0.005;
end_time = 25;
loop = 1;
maxloops = ceil(end_time/deltat);

% this struct can be used to evolve what the UVMS has to do
mission.phase = 1;
mission.phase_time = 0;

% Rotation matrix to convert coordinates between Unity and the <w> frame
% do not change
wuRw = rotation(0,-pi/2,pi/2);
vRvu = rotation(-pi/2,0,-pi/2);

% pipe parameters
u_pipe_center = [-10.66 31.47 -1.94]'; % in unity coordinates
pipe_center = wuRw'*u_pipe_center;     % in world frame coordinates
pipe_radius = 0.3;

% UDP Connection with Unity viewer v2
uArm = udp('127.0.0.1',15000,'OutputDatagramPacketSize',28);
uVehicle = udp('127.0.0.1',15001,'OutputDatagramPacketSize',24);
fopen(uVehicle);
fopen(uArm);

% Preallocation
plt = InitDataPlot(maxloops);

% initialize uvms structure
uvms = InitUVMS('DexROV');
% uvms.q 
% Initial joint positions. You can change these values to initialize the simulation with a 
% different starting position for the arm
uvms.q = [-0.0031 1.2586 0.0128 -1.2460 0.0137 0.0853-pi/2 0.0137]';
% uvms.p
% initial position of the vehicle
% the vector contains the values in the following order
% [x y z r(rot_x) p(rot_y) y(rot_z)]
% RPY angles are applied in the following sequence
% R(rot_x, rot_y, rot_z) = Rz (rot_z) * Ry(rot_y) * Rx(rot_x)
uvms.p = [-1.9379 10.4813-6.1 -29.7242-0.1 0 0 0]';

% initial goal position definition
% slightly over the top of the pipe
distanceGoalWrtPipe = 0.4;
uvms.goalPosition = pipe_center + (pipe_radius + distanceGoalWrtPipe)*[0 0 1]';
uvms.wRg = rotation(pi,0,0);
uvms.wTg = [uvms.wRg uvms.goalPosition; 0 0 0 1];

vehicle_distanceGoalWrtPipe = 2;
uvms.vehicle_goalPosition = pipe_center + (pipe_radius + vehicle_distanceGoalWrtPipe)*[0 0 1]';
uvms.wRggv = rotation(0,0,0);
uvms.wTggv = [uvms.wRggv uvms.vehicle_goalPosition; 0 0 0 1];

uvms.prefe_values=[-0.0031,1.2586,0.0128,-1.246,0,0,0];

% defines the tool control point
uvms.eTt = eye(4);
tic
for t = 0:deltat:end_time
    % update all the involved variables
    uvms = UpdateTransforms(uvms);
    uvms = ComputeJacobians(uvms);
    uvms = ComputeTaskReferences(uvms, mission);
    uvms = ComputeActivationFunctions(uvms, mission);
   
    % main kinematic algorithm initialization
    % rhop order is [qdot_1, qdot_2, ..., qdot_7, xdot, ydot, zdot, omega_x, omega_y, omega_z]
    
    % TPIK 1 (centralized)
    
    rhop = zeros(13,1);
    Qp = eye(13); 
    % add all the other tasks here!
    % the sequence of iCAT_task calls defines the priority
    %[Qp, rhop] = iCAT_task(uvms.A.mu,   uvms.Jmu,   Qp, rhop, uvms.xdot.mu, 0.000001, 0.0001, 10);
    %[Qp, rhop] = iCAT_task(uvms.A.ha,   uvms.Jha,   Qp, rhop, uvms.xdot.ha, 0.0001,   0.01, 10);
    %[Qp, rhop] = iCAT_task(uvms.A.t,    uvms.Jt,    Qp, rhop, uvms.xdot.t,  0.0001,   0.01, 10);
    [Qp, rhop] = iCAT_task(uvms.A.jl,    uvms.Jjl, Qp, rhop, uvms.xdot.jl,  0.0001,   0.01, 10);
    %[Qp, rhop] = iCAT_task(uvms.A.vcv,    uvms.Jvcv, Qp, rhop, uvms.xdot.vcv,  0.0001,   0.01, 10);
    [Qp, rhop] = iCAT_task(uvms.A.t,    uvms.Jt, Qp, rhop, uvms.xdot.t,  0.0001,   0.01, 10);
    %[Qp, rhop] = iCAT_task(uvms.A.prefe_value,    uvms.Jprefe_value, Qp, rhop, uvms.xdot.prefe_value,  0.0001,   0.01, 10);
    %[Qp, rhop] = iCAT_task(uvms.A.ha,    uvms.Jha,  Qp, rhop, uvms.xdot.ha,  0.0001,   0.01, 10);
    %[Qp, rhop] = iCAT_task(uvms.A.a,     uvms.Ja,   Qp, rhop, uvms.xdot.a,  0.0001,   0.01, 10);
    %[Qp, rhop] = iCAT_task(uvms.A.ma,    uvms.Jma,  Qp, rhop, uvms.xdot.ma,  0.0001,   0.01, 10);
    [Qp, rhop] = iCAT_task(uvms.A.v_l,    uvms.Jv_l, Qp, rhop, uvms.xdot.v_l,  0.0001,   0.01, 10);
    [Qp, rhop] = iCAT_task(uvms.A.v_a,    uvms.Jv_a, Qp, rhop, uvms.xdot.v_a,  0.0001,   0.01, 10);
    [Qp, rhop] = iCAT_task(eye(13),     eye(13),    Qp, rhop, zeros(13,1),  0.0001,   0.01, 10);    % this task should be the last one
    
    
    vehicle_ref_velocity = rhop(8:13);
    % TPIK 2 (arm optimized on current vehicle velocity)
    
    rhop = zeros(13,1);
    Qp = eye(13); 
    [Qp, rhop] = iCAT_task(uvms.A.vcv,    uvms.Jvcv, Qp, rhop, uvms.xdot.vcv,  0.0001,   0.01, 10);
    [Qp, rhop] = iCAT_task(uvms.A.jl,    uvms.Jjl, Qp, rhop, uvms.xdot.jl,  0.0001,   0.01, 10);
    [Qp, rhop] = iCAT_task(uvms.A.t,    uvms.Jt, Qp, rhop, uvms.xdot.t,  0.0001,   0.01, 10);
    %[Qp, rhop] = iCAT_task(uvms.A.prefe_value,    uvms.Jprefe_value, Qp, rhop, uvms.xdot.prefe_value,  0.0001,   0.01, 10);
    %[Qp, rhop] = iCAT_task(uvms.A.ha,    uvms.Jha,  Qp, rhop, uvms.xdot.ha,  0.0001,   0.01, 10);
    %[Qp, rhop] = iCAT_task(uvms.A.a,     uvms.Ja,   Qp, rhop, uvms.xdot.a,  0.0001,   0.01, 10);
    %[Qp, rhop] = iCAT_task(uvms.A.ma,    uvms.Jma,  Qp, rhop, uvms.xdot.ma,  0.0001,   0.01, 10);
    %[Qp, rhop] = iCAT_task(uvms.A.v_l,    uvms.Jv_l, Qp, rhop, uvms.xdot.v_l,  0.0001,   0.01, 10);
    %[Qp, rhop] = iCAT_task(uvms.A.v_a,    uvms.Jv_a, Qp, rhop, uvms.xdot.v_a,  0.0001,   0.01, 10);
    [Qp, rhop] = iCAT_task(eye(13),     eye(13),    Qp, rhop, zeros(13,1),  0.0001,   0.01, 10);    % this task should be the last one
    
    
    % get the two variables for integration
    uvms.q_dot = rhop(1:7);
    uvms.p_dot = vehicle_ref_velocity;
    
    %disturbances
    uvms.p_dot = uvms.p_dot + 0.5*sin(t);
    
    % Integration
	uvms.q = uvms.q + uvms.q_dot*deltat;
    % beware: p_dot should be projected on <v>
    uvms.p = integrate_vehicle(uvms.p, uvms.p_dot, deltat);
    
    % check if the mission phase should be changed
    [uvms, mission] = UpdateMissionPhase(uvms, mission);
    
    % send packets to Unity viewer
    SendUdpPackets(uvms,wuRw,vRvu,uArm,uVehicle);
        
    % collect data for plots
    plt = UpdateDataPlot(plt,uvms,t,loop);
    loop = loop + 1;
   
    % add debug prints here
    if (mod(t,0.1) == 0)
        t
        %uvms.p'
    end
    
    % enable this to have the simulation approximately evolving like real
    % time. Remove to go as fast as possible
    SlowdownToRealtime(deltat);
end

fclose(uVehicle);
fclose(uArm);

PrintPlot(plt);
