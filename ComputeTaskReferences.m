function [uvms] = ComputeTaskReferences(uvms, mission)
% compute the task references here

% reference for tool-frame position control task
[ang, lin] = CartError(uvms.vTg , uvms.vTt);
uvms.xdot.t = 0.8 * [ang; lin];
% limit the requested velocities...
uvms.xdot.t(1:3) = Saturate(uvms.xdot.t(1:3), 0.8);
uvms.xdot.t(4:6) = Saturate(uvms.xdot.t(4:6), 0.8);

% reference for vehicle position control task - projected on the world
% frame
[ang_v, lin_v] = CartError(uvms.wTggv , uvms.wTv);
uvms.xdot.v_l=0.8*lin_v;
uvms.xdot.v_a=0.8*ang_v;
uvms.xdot.v_l = Saturate(uvms.xdot.v_l, 0.8);
uvms.xdot.v_a = Saturate(uvms.xdot.v_a, 0.8);

% reference for the vehicle constrained velocity task
uvms.xdot.vcv = zeros(6,1);

%joint limits task

%calcolo la differenza tra la mia q e il limite minimo e massimo che devo raggiungere
uvms.diff_jlmin=uvms.q-uvms.jlmin;
uvms.diff_jlmax=uvms.q-uvms.jlmax;

uvms.abs_diff_jlmin=abs(uvms.diff_jlmin);
uvms.abs_diff_jlmax=abs(uvms.diff_jlmax);

uvms.abs_diff=min(uvms.abs_diff_jlmin,uvms.abs_diff_jlmax);
%Successivamente per ogni giunto andrò ad imporre una desiderata velocità
%per farmi allontanare.

%questo ciclo va da 1 a 7 ed esamina tutti i giunti
for i=1:size(uvms.diff_jlmin,1)
    
    %Qui si potrebbe aggiungere un controllo per dirgli che tutti sti conti
    %li deve fare solo se è molto vicino al limite
    
    %per ogni giunto tengo in considerazione se è più vicino al limite
    %minimo o massimo. E di conseguenza imposto la velocità di
    %allontanamento
    if(uvms.abs_diff_jlmin(i,1)<uvms.abs_diff_jlmax(i,1))
        uvms.xdot.jl(i,1)=1;
    else
        uvms.xdot.jl(i,1)=-1;
    end
    
    
end
uvms.xdot.jl=uvms.xdot.jl*1; 


%preferred shape
uvms.q;
uvms.diff_prefe_value=uvms.q-uvms.prefe_values';
%uvms.xdot.prefe_value=-sign(uvms.diff_prefe_value)*0.1;
uvms.xdot.prefe_value= 0.8*(zeros(7,1)+uvms.diff_prefe_value);
uvms.xdot.prefe_value = Saturate(uvms.xdot.prefe_value, 0.8);
%fprintf('%f %f %f %f\n',uvms.diff_prefe_value(1),uvms.diff_prefe_value(2),uvms.diff_prefe_value(3),uvms.diff_prefe_value(4));