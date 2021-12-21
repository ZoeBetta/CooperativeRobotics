function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here

% arm tool position control
% always active
uvms.A.t = eye(6);
uvms.Ap.ha = 1;
uvms.Ap.a = 1;

%horizontal attitude
uvms.A.ha = IncreasingBellShapedFunction(0.2, 0.4, 0, 1, norm(uvms.v_rho_ha))*uvms.Ap.ha;

%altitude
uvms.A.a = 1*uvms.Ap.a;