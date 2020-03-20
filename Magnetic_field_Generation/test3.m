wo=rand(3,1);
global poles
poles=rand(6,1);

h=@gain_matrix;
A=state_matrix(wo);

K=testfunc(wo,poles)
