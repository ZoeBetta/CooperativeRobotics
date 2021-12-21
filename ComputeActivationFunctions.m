function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here

% arm tool position control
% always active
uvms.A.t = eye(6);
uvms.A.v_l= eye(3);
uvms.A.v_a=eye(3);
uvms.A.ha = 1;