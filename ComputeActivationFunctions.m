function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here
    switch mission.phase
        case 1  
            %uvms.Ap.ha = 1;
            uvms.Ap.ma = 1;
            %uvms.Ap.a = 0;
        case 2
            %uvms.Ap.ha = 1;
            uvms.Ap.ma = 1;
            %uvms.Ap.a = 1; %IncreasingBellShapedFunction(0, 1, 0, 1, mission.phase_time);
    end
% arm tool position control
% always active
uvms.A.t = eye(6);

uvms.A.v_l= eye(3);
uvms.A.v_a=eye(3);


%minimum altitude
uvms.Ap.ma = 1;
uvms.A.ma = DecreasingBellShapedFunction(0.8, 1, 0, 1, uvms.a)*uvms.Ap.ma; %1.3 è la distanza massima
