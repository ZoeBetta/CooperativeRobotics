function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here
    switch mission.phase
        case 1  
            uvms.Ap.ha = 1;
            uvms.Ap.ma = 1;
            uvms.Ap.t=eye(6);
            uvms.Ap.v_l=eye(3);
            uvms.Ap.v_a=eye(3);
        case 2
    end
% arm tool position control
% always active
uvms.A.t = eye(6)*uvms.Ap.t;
uvms.A.v_l= eye(3)*uvms.Ap.v_l;
uvms.A.v_a=eye(3)*uvms.Ap.v_a;


%horizontal attitude
uvms.A.ha = IncreasingBellShapedFunction(0.2, 0.4, 0, 1, norm(uvms.v_rho_ha))*uvms.Ap.ha;


%minimum altitude
uvms.A.ma = DecreasingBellShapedFunction(4.5, 5, 0, 1, uvms.a)*uvms.Ap.ma; %1 è la distanza massima

