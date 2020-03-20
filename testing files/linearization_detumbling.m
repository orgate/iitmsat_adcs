
syms X b bt w0 I K;
syms q1 q2 q3 q4 w1 w2 w3;
syms Ix Iy Iz;
syms b1 b2 b3;
syms bt1 bt2 bt3;

X=[q1;q2;q3;q4;w1;w2;w3];
I=[Ix 0 0;0 Iy 0;0 0 Iz];
b=[b1;b2;b3];
bt=[bt1;bt2;bt3];
p=input('Enter Linearization Point: (7x1 vector)');
J=jacobian(detumbling_nonlinear(X,b,bt,w0,I,K),X)
A=subs(J, X, p)
