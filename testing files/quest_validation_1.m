%%%%%%%%
% The following m file validates the use of quest algorithm for use of 
%quest algorithm in finding error rotation matrix

%%%% Validation procedure %%%%%%%%
% We assume %We let dvb=(I-E1)*vb, and dwb=(I-E1)*wb, where E1 is some arbitrary rotation matrix.
% On applying wahaba's problem condition the rotation error matrix is E1

%Assume a true random rotation matrix
a=360*rand(1);b=360*rand(1);c=360*rand(1); %Euler angs of true orientation
 A=[cosd(a) -sind(a) 0;sind(a) cosd(a) 0;0 0 1]...
    *[cosd(b) 0 sind(b);0 1 0;-sind(b) 0 cosd(b)]*[1 0 0;0 cosd(c) -sind(c);...
     0 sind(c) cosd(c)];
 
 % Assume the rotation error matrix
 ea=.1*pi/180*rand(1);eb=.1*pi/180*rand(1);ec=.1*pi/180*rand(1);
 E=[cos(ea) -sin(ea) 0;sin(ea) cos(ea) 0;0 0 1]...
    *[cos(eb) 0 sin(eb);0 1 0;-sin(eb) 0 cos(eb)]*[1 0 0;0 cos(ec) -sin(ec);...
     0 sin(ec) cos(ec)];
 
 % Generate vectors in orbit frame
vo=rand(3,1);
vo=vo/sqrt(sum(vo.*vo))
vb=A*vo;% Atual vector1 in body frame
wo=rand(3,1);
wo=wo/sqrt(sum(wo.*wo))
wb=A*wo;% Actual vector2 in body frame

%%%%%%%%%%%%%%Calculation of Rotation matrix with error in sensor values

dvb=vb-E*vb; % error assumption
dwb=wb-E*wb; % error assumption
dvo=[0;0;0];
dwo=[0;0;0];
vdb=vb+dvb;
wdb=wb+dwb;
vdo=vo+dvo;
wdo=wo+dwo;

%%%Calculation of rotation matrix with sensor errors
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
[E2, lambda]=eig(Ke);
[val1 posn1]=max(lambda);
[val2 posn2]=max(val1);
Ad=Quaternion_to_DCM(E2(:,posn2));% Rottaion matrix with sensor error

Error_Rot=Ad*A';% Error in rotation matrix

check=E*Error_Rot; %check for check=I

