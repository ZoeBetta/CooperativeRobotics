function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here
    switch mission.phase
        case 1  
            uvms.Ap.ha = 1;
            uvms.Ap.ma = 1;
            %uvms.Ap.a = 0;
            
            uvms.Ap.v_l = eye(3);
            uvms.Ap.v_a = eye(3);
            uvms.Ap.vcv = zeros(6);
            uvms.Ap.t=zeros(6);
            uvms.Ap.jl=eye(7);
        case 2
            uvms.Ap.ha = 0;
            uvms.Ap.ma = 0;
           % uvms.Ap.a = 0; 
            uvms.Ap.v_l = zeros(3);
            uvms.Ap.v_a = zeros(3);
            uvms.Ap.vcv = eye(6);
            uvms.Ap.t=eye(6);
            uvms.Ap.jl=eye(7);
            
    end
% arm tool position control
% always active
uvms.A.t = eye(6)*uvms.Ap.t;

uvms.A.v_l= eye(3)*uvms.Ap.v_l;
uvms.A.v_a= eye(3)*uvms.Ap.v_a;


%horizontal attitude
uvms.A.ha = IncreasingBellShapedFunction(0.2, 0.4, 0, 1, norm(uvms.v_rho_ha))*uvms.Ap.ha;
%uvms.A.ha = 1*uvms.Ap.ha;

%minimum altitude
uvms.Ap.ma = 1;
uvms.A.ma = DecreasingBellShapedFunction(0.5, 1, 0, 1, uvms.a)*uvms.Ap.ma; %1 è la distanza massima


%altitude
uvms.A.a = 1*uvms.Ap.a;


%vehicle constrained velocity
uvms.A.vcv = eye(6)*uvms.Ap.vcv;

%arm joints limits

% se il valore assoluto della differenza tra i valori dei giunti e i loro limiti è compresa tra 0
% e 0.01 allora attiva la task
for i=1:size(uvms.diff_jlmin,1)
    uvms.A.jl(i,i) = DecreasingBellShapedFunction(0, 0.01, 0, 1,uvms.abs_diff(i,1))*uvms.Ap.jl(i,i);
    
    
    %decommentare qui sotto per avere un pò di debug.
    if uvms.A.jl(i,i)>0 && i==7
        fprintf('i:%f q:%f  diffmin:%f diffmax:%f  jlmin:%f jlmax:%f xdot:%f\n',i,uvms.q(i,1),uvms.diff_jlmin(i,1),uvms.diff_jlmax(i,1),uvms.jlmin(i,1),uvms.jlmax(i,1),uvms.xdot.jl(i,1))
    end
end
