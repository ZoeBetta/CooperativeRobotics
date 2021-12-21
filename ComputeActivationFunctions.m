function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here
    switch mission.phase
        case 1  
            uvms.Ap.ha = 1;
            uvms.Ap.ma = 1;
            uvms.Ap.a = 0;
        case 2
            uvms.Ap.ha = 1;
            uvms.Ap.ma = 0;
            uvms.Ap.a = 1; %IncreasingBellShapedFunction(0, 1, 0, 1, mission.phase_time);
    end
% arm tool position control
% always active
uvms.A.t = eye(6);

uvms.A.v_l= eye(3);
uvms.A.v_a=eye(3);


%horizontal attitude
uvms.A.ha = IncreasingBellShapedFunction(0.2, 0.4, 0, 1, norm(uvms.v_rho_ha))*uvms.Ap.ha;


%minimum altitude
uvms.Ap.ma = 1;
uvms.A.ma = DecreasingBellShapedFunction(0.5, 1, 0, 1, uvms.a)*uvms.Ap.ma; %1 è la distanza massima


%altitude
uvms.A.a = 1*uvms.Ap.a;