function K=gain_matrix(wo,poles)
%this function generates the gain matrix with given poles
Isatinv=[18.2 0 0;0 21.43 0;0 0 18.12];
B=cat(2,Isatinv,zeros(3,3));
G=cat(1,B,zeros(3,6));
A=@state_matrix;
F=A(wo);

K=place(F,G,poles);

