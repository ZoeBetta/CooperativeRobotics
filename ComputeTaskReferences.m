function [uvms] = ComputeTaskReferences(uvms, mission)
% compute the task references here

% reference for tool-frame position control task
[ang, lin] = CartError(uvms.vTg , uvms.vTt);
uvms.xdot.t = 0.2 * [ang; lin];
% limit the requested velocities...
uvms.xdot.t(1:3) = Saturate(uvms.xdot.t(1:3), 0.2);
uvms.xdot.t(4:6) = Saturate(uvms.xdot.t(4:6), 0.2);

% reference for vehicle position control task - projected on the world
% frame
[ang_v, lin_v] = CartError(uvms.wTggv , uvms.wTv);
uvms.xdot.v_l=0.8*lin_v;
uvms.xdot.v_a=0.8*ang_v;
uvms.xdot.v_l = Saturate(uvms.xdot.v_l, 0.8);
uvms.xdot.v_a = Saturate(uvms.xdot.v_a, 0.8);


% reference for the horizontal attitude task
uvms.xdot.ha = 0.2 *(0 - norm(uvms.v_rho_ha));
uvms.xdot.ha = Saturate(uvms.xdot.ha, 0.5);

% reference for the minimum altitude task
uvms.xdot.ma = 0.8 * (5 - uvms.a);
uvms.xdot.ma = Saturate(uvms.xdot.ma, 0.8);
