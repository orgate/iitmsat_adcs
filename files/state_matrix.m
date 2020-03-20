function [A]=state_matrix(wo)
% This function generates the linearized state matrix at a given wo(set
% angular velocity)

Isat=7/12*[20*2+30^2 0 0;0 2*20^2 0; 0 0 20*2+30^2]*10^-4;

a=Isat*wo;

P=[inv(Isat)*(skew_sym(a)-skew_sym(wo)*Isat) zeros(3,3);];
Q=[0.5*eye(3,3) -skew_sym(wo)];
A=cat(1,P,Q);
A=double(A);


end