function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here
    switch mission.phase
        case 1  
            uvms.Ap.v_l = eye(3);
            uvms.Ap.v_a = eye(3);
            uvms.Ap.vcv = zeros(6);
            uvms.Ap.t=zeros(6);
            uvms.Ap.jl=eye(7);
            uvms.Ap.prefe_value=zeros(7);
        case 2
            uvms.Ap.v_l = eye(3);
            uvms.Ap.v_a = eye(3);
            uvms.Ap.vcv = eye(6);
            uvms.Ap.t=eye(6);
            uvms.Ap.jl=eye(7);
            uvms.Ap.prefe_value=eye(7);
            
    end
% arm tool position control
% always active
uvms.A.t = eye(6)*uvms.Ap.t;

uvms.A.v_l= eye(3)*uvms.Ap.v_l;
uvms.A.v_a= eye(3)*uvms.Ap.v_a;

%vehicle constrained velocity
uvms.A.vcv = eye(6)*uvms.Ap.vcv;

%arm joints limits

% se il valore assoluto della differenza tra i valori dei giunti e i loro limiti è compresa tra 0
% e 0.01 allora attiva la task
for i=1:size(uvms.diff_jlmin,1)
    uvms.A.jl(i,i) = DecreasingBellShapedFunction(0, 0.01, 0, 1,uvms.abs_diff(i,1))*uvms.Ap.jl(i,i);
    
    
    %decommentare qui sotto per avere un pò di debug.
    if uvms.A.jl(i,i)>0 && i==7
        %fprintf('i:%f q:%f  diffmin:%f diffmax:%f  jlmin:%f jlmax:%f xdot:%f\n',i,uvms.q(i,1),uvms.diff_jlmin(i,1),uvms.diff_jlmax(i,1),uvms.jlmin(i,1),uvms.jlmax(i,1),uvms.xdot.jl(i,1))
    end
end

%Preferred shapes

uvms.Ap.prefe_value(5,5)=0;uvms.Ap.prefe_value(6,6)=0;uvms.Ap.prefe_value(7,7)=0;
uvms.A.prefe_value=uvms.Ap.prefe_value;