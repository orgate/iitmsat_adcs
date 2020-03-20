%We equate all errors to zero. We should get t1=t2=t3=0 in both cases of approximation. AA's
%should be non-singular.
a=360*rand(1);b=360*rand(1);c=360*rand(1); %Euler angs of true orientation
 A=[cosd(a) -sind(a) 0;sind(a) cosd(a) 0;0 0 1]...
    *[cosd(b) 0 sind(b);0 1 0;-sind(b) 0 cosd(b)]*[1 0 0;0 cosd(c) -sind(c);...
     0 sind(c) cosd(c)];
 
 ea=.1*rand(1);eb=.1*rand(1);ec=.1*rand(1);
 E=[cos(ea) -sin(ea) 0;sin(ea) cos(ea) 0;0 0 1]...
    *[cos(eb) 0 sin(eb);0 1 0;-sin(eb) 0 cos(eb)]*[1 0 0;0 cos(ec) -sin(ec);...
     0 sin(ec) cos(ec)];
 
w1=rand(3,1);
w1=w1/sqrt(sum(w1.*w1))
v1=A*w1
w2=rand(3,1)
w2=w2/sqrt(sum(w2.*w2))
v2=A*w2

dv1=zeros(3,1)
dw1=E*w1-w1
dv2=zeros(3,1)
dw2=E*w2-w2



AA1=AMatrix_I(w1(1),w1(2),w1(3),w2(1),w2(2),w2(3),dw1(1),dw1(2),dw1(3),dw2(1),dw2(2),...
    dw2(3),A(1,1),A(1,2),A(1,3),A(2,1),A(2,2),A(2,3),A(3,1),A(3,2),A(3,3),dv1(1),dv1(2),dv1(3),dv2(1),dv2(2),dv2(3));
bb1=bMatrix_I(w1(1),w1(2),w1(3),w2(1),w2(2),w2(3),dw1(1),dw1(2),dw1(3),dw2(1),dw2(2),...
    dw2(3),A(1,1),A(1,2),A(1,3),A(2,1),A(2,2),A(2,3),A(3,1),A(3,2),A(3,3),dv1(1),dv1(2),dv1(3),dv2(1),dv2(2),dv2(3))

AA2=AMatrix_II(w1(1),w1(2),w1(3),w2(1),w2(2),w2(3),dw1(1),dw1(2),dw1(3),dw2(1),dw2(2),...
    dw2(3),A(1,1),A(1,2),A(1,3),A(2,1),A(2,2),A(2,3),A(3,1),A(3,2),A(3,3),dv1(1),dv1(2),dv1(3),dv2(1),dv2(2),dv2(3));
bb2=bMatrix_II(w1(1),w1(2),w1(3),w2(1),w2(2),w2(3),dw1(1),dw1(2),dw1(3),dw2(1),dw2(2),...
    dw2(3),A(1,1),A(1,2),A(1,3),A(2,1),A(2,2),A(2,3),A(3,1),A(3,2),A(3,3),dv1(1),dv1(2),dv1(3),dv2(1),dv2(2),dv2(3))

A1=det(AA1)
A2=det(AA2)

tt1=AA1\bb1
tt2=AA2\bb2

t1=sqrt(sum(tt1.*tt1))
t2=sqrt(sum(tt2.*tt2))

[C V]=eigs(E)
t=abs(asin(imag(V(1,1))))

[t t1 t2]

