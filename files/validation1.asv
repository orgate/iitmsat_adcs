%We let dv1=(I-E1)*v1, and dv2=(I-E1)*v2, where E1 is some arbitrary rotation matrix.


clear all;
clc;
a=360*rand(1);b=360*rand(1);c=360*rand(1); %Euler angs of true orientation
 A=[cosd(a) -sind(a) 0;sind(a) cosd(a) 0;0 0 1]...
    *[cosd(b) 0 sind(b);0 1 0;-sind(b) 0 cosd(b)]*[1 0 0;0 cosd(c) -sind(c);...
     0 sind(c) cosd(c)];
 
 ea=.1*pi/180*rand(1);eb=.1*pi/180*rand(1);ec=.1*pi/180*rand(1);
 E=[cos(ea) -sin(ea) 0;sin(ea) cos(ea) 0;0 0 1]...
    *[cos(eb) 0 sin(eb);0 1 0;-sin(eb) 0 cos(eb)]*[1 0 0;0 cos(ec) -sin(ec);...
     0 sin(ec) cos(ec)];
 
w1=rand(3,1);
w1=w1/sqrt(sum(w1.*w1))
v1=A*w1
w2=rand(3,1)
w2=w2/sqrt(sum(w2.*w2))
v2=A*w2

dv1=v1-E*v1
dw1=zeros(3,1)
dv2=v2-E*v2
dw2=zeros(3,1)


%%%%%%Considering only sine term in exponential rotation form%%%%
AA1=AMatrix_I(w1(1),w1(2),w1(3),w2(1),w2(2),w2(3),dw1(1),dw1(2),dw1(3),dw2(1),dw2(2),...
    dw2(3),A(1,1),A(1,2),A(1,3),A(2,1),A(2,2),A(2,3),A(3,1),A(3,2),A(3,3),dv1(1),dv1(2),dv1(3),dv2(1),dv2(2),dv2(3));
bb1=bMatrix_I(w1(1),w1(2),w1(3),w2(1),w2(2),w2(3),dw1(1),dw1(2),dw1(3),dw2(1),dw2(2),...
    dw2(3),A(1,1),A(1,2),A(1,3),A(2,1),A(2,2),A(2,3),A(3,1),A(3,2),A(3,3),dv1(1),dv1(2),dv1(3),dv2(1),dv2(2),dv2(3))
%%%%%%Considering both cos and sine terms in exponential rotation form%%%%
AA2=AMatrix_II(w1(1),w1(2),w1(3),w2(1),w2(2),w2(3),dw1(1),dw1(2),dw1(3),dw2(1),dw2(2),dw2(3),...
    A(1,1),A(1,2),A(1,3),A(2,1),A(2,2),A(2,3),A(3,1),A(3,2),A(3,3),dv1(1),dv1(2),dv1(3),dv2(1),dv2(2),dv2(3));
bb2=bMatrix_II(w1(1),w1(2),w1(3),w2(1),w2(2),w2(3),dw1(1),dw1(2),dw1(3),dw2(1),dw2(2),dw2(3),...
    A(1,1),A(1,2),A(1,3),A(2,1),A(2,2),A(2,3),A(3,1),A(3,2),A(3,3),dv1(1),dv1(2),dv1(3),dv2(1),dv2(2),dv2(3));

A1=det(AA1);
A2=det(AA2);

tt1=AA1\bb1;
tt2=AA2\bb2;

t1=sqrt(sum(tt1.*tt1));
t2=sqrt(sum(tt2.*tt2));

[C V]=eigs(E);

t=abs(atan2(imag(V(2,2)),abs(real(V(2,2)))));

% results=[t t1 t2]
abs(dv1./v1*100)
abs(dv2./v2*100)
err=abs([(t1-t)/t*100 (t2-t)/t*100])

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

a=360*rand(1);b=360*rand(1);c=360*rand(1); %Euler angs of true orientation
 A=[cosd(a) -sind(a) 0;sind(a) cosd(a) 0;0 0 1]...
    *[cosd(b) 0 sind(b);0 1 0;-sind(b) 0 cosd(b)]*[1 0 0;0 cosd(c) -sind(c);...
     0 sind(c) cosd(c)];

vo=w1;
vb=A*vo;
wo=w2;
wb=A*wo;
dvb=vb-E*vb;
dwb=wb-E*wb;
dvo=[0;0;0];
dwo=[0;0;0];
vdb=vb+dvb;
wdb=wb+dwb;
vdo=vo+dvo;
wdo=wo+dwo;
%%%%%%%%%%%%%%Calculation of Rotation matrix with no error in sensor values%%%%%%%%
K=zeros(4,4);
s=[1;1];%weights
B=s(1)*(vb*vo')+s(2)*(wb*wo');
S=B+B';
Z=[B(2,3)-B(3,2),B(3,1)-B(1,3),B(1,2)-B(2,1)]';
sig=trace(B);
K(1:3,1:3)=S-sig*eye(3);
K(4,1:3)=Z';
K(1:3,4)=Z;
K(4,4)=sig;
[E1, lambda]=eig(K)
[val1 posn1]=max(lambda);
[val2 posn2]=max(val1)
Ad=Quaternion_to_DCM(E1(:,posn2))
A-Ad
%%%%%%%%%%%%%%Calculation of Rotation matrix with error in sensor values
Ke=zeros(4,4);
se=[1;1];%weights
Be=se(1)*(vdb*vdo')+s(2)*(wdb*wdo');
Se=Be+Be';
Ze=[Be(2,3)-Be(3,2),Be(3,1)-Be(1,3),Be(1,2)-Be(2,1)]';
sige=trace(Be);
Ke(1:3,1:3)=Se-sige*eye(3);
Ke(4,1:3)=Ze';
Ke(1:3,4)=Ze;
Ke(4,4)=sige;
[E2, lambda]=eig(Ke)
[val1 posn1]=max(lambda);
[val2 posn2]=max(val1)
Ad=Quaternion_to_DCM(E2(:,posn2))
Error_Rot=Ad*A';

[C V]=eigs(Error_Rot);

te=abs(atan2(imag(V(2,2)),abs(real(V(2,2)))));

results=[t t1 t2 te]


