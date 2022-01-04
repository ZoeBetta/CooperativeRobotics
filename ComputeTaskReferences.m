function [uvms] = ComputeTaskReferences(uvms, mission)
% compute the task references here

% reference for tool-frame position control task
[ang, lin] = CartError(uvms.vTg , uvms.vTt);
uvms.xdot.t = 0.2 * [ang; lin];
% limit the requested velocities...
uvms.xdot.t(1:3) = Saturate(uvms.xdot.t(1:3), 0.2);
uvms.xdot.t(4:6) = Saturate(uvms.xdot.t(4:6), 0.2);

% reference for the horizontal attitude task
uvms.xdot.ha = 0.2 *(0 - norm(uvms.v_rho_ha));
uvms.xdot.ha = Saturate(uvms.xdot.ha, 0.5);
% reference for the altitude task
uvms.xdot.a = 0.2 * (0 - uvms.a);
uvms.xdot.a = Saturate(uvms.xdot.a, 0.5);