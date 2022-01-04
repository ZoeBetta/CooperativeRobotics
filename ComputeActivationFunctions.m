function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here
    switch mission.phase
        case 1  
            uvms.Ap.ha = 1;
            uvms.Ap.ma = 1;
            uvms.Ap.a = 0;
            uvms.Ap.o=0;
            uvms.Ap.v_l = eye(3);
            uvms.Ap.v_a = eye(3);
            uvms.Ap.vcv = zeros(6);
            uvms.Ap.t=zeros(6);
        case 2
            uvms.Ap.ha = 1;
            uvms.Ap.ma = 0;
            uvms.Ap.a = 1; 
            uvms.Ap.v_l = zeros(3);
            uvms.Ap.v_a = zeros(3);
            uvms.Ap.vcv = zeros(6);
            uvms.Ap.t=zeros(6);
            uvms.Ap.o=1;
        case 3
            uvms.Ap.ha = 0;
            uvms.Ap.ma = 0;
            uvms.Ap.a = 0; 
            uvms.Ap.v_l = zeros(3);
            uvms.Ap.v_a = zeros(3);
            uvms.Ap.vcv = eye(6);
            uvms.Ap.t=eye(6);
            uvms.Ap.o=0;
            
    end
% arm tool position control
% always active
uvms.A.t = eye(6)*uvms.Ap.t;

uvms.A.v_l= eye(3)*uvms.Ap.v_l;
uvms.A.v_a= eye(3)*uvms.Ap.v_a;

%horizontal attitude
%uvms.A.ha = IncreasingBellShapedFunction(0.2, 0.4, 0, 1, norm(uvms.v_rho_ha))*uvms.Ap.ha;
uvms.A.ha = IncreasingBellShapedFunction(0.2, 0.4, 0, 1, norm(uvms.v_rho_ha))*uvms.Ap.ha;
%uvms.A.ha = 1*uvms.Ap.ha;

%minimum altitude
uvms.A.ma = DecreasingBellShapedFunction(0.5, 1, 0, 1, uvms.a)*uvms.Ap.ma; %1 è la distanza massima

%altitude
uvms.A.a = 1*uvms.Ap.a;

%vehicle constrained velocity
uvms.A.vcv = eye(6)*uvms.Ap.vcv;

%orientation equality, I want the two directions to be exactly the same
uvms.A.o=1*uvms.Ap.o;