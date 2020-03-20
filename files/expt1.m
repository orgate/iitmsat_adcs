a=360*rand(1);b=360*rand(1);c=360*rand(1); %Euler angs of true orientation
 A=[cosd(a) -sind(a) 0;sind(a) cosd(a) 0;0 0 1]...
    *[cosd(b) 0 sind(b);0 1 0;-sind(b) 0 cosd(b)]*[1 0 0;0 cosd(c) -sind(c);...
     0 sind(c) cosd(c)];
w1=rand(3,1);
w1=w1/sqrt(sum(v1.*v1))

v1=A*w1

dv1=rand(3,1)/10
dw1=rand(3,1)/10

% dv1=zeros(3,1)
% dw1=zeros(3,1)
% dv2=zeros(3,1)
% dw2=zeros(3,1)

w2=rand(3,1)
w2=w2/sqrt(sum(v2.*v2))

v2=A*w2

dv2=rand(3,1)/10
dw2=rand(3,1)/10
% 
AA1=AMatrix_I(w1(1),w1(2),w1(3),w2(1),w2(2),w2(3),dw1(1),dw1(2),dw1(3),dw2(1),dw2(2),dw2(3),...
    A(1,1),A(1,2),A(1,3),A(2,1),A(2,2),A(2,3),A(3,1),A(3,2),A(3,3),dv1(1),dv1(2),dv1(3),dv2(1),dv2(2),dv2(3));
bb1=bMatrix_I(w1(1),w1(2),w1(3),w2(1),w2(2),w2(3),dw1(1),dw1(2),dw1(3),dw2(1),dw2(2),dw2(3),...
    A(1,1),A(1,2),A(1,3),A(2,1),A(2,2),A(2,3),A(3,1),A(3,2),A(3,3),dv1(1),dv1(2),dv1(3),dv2(1),dv2(2),dv2(3));

AA2=AMatrix_II(w1(1),w1(2),w1(3),w2(1),w2(2),w2(3),dw1(1),dw1(2),dw1(3),dw2(1),dw2(2),dw2(3),...
    A(1,1),A(1,2),A(1,3),A(2,1),A(2,2),A(2,3),A(3,1),A(3,2),A(3,3),dv1(1),dv1(2),dv1(3),dv2(1),dv2(2),dv2(3));
bb2=bMatrix_II(w1(1),w1(2),w1(3),w2(1),w2(2),w2(3),dw1(1),dw1(2),dw1(3),dw2(1),dw2(2),dw2(3),...
    A(1,1),A(1,2),A(1,3),A(2,1),A(2,2),A(2,3),A(3,1),A(3,2),A(3,3),dv1(1),dv1(2),dv1(3),dv2(1),dv2(2),dv2(3));

tt1=AA1\bb1;
tt2=AA2\bb2;

t1=sqrt(sum(tt1.*tt1));
t2=sqrt(sum(tt2.*tt2));
