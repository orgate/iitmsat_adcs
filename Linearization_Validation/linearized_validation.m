function T_F=linearized_validation
% This is the supporting function to validate the linearization of
% satellite equations
options = odeset('RelTol',1e-7,'AbsTol',(10^-1)*[1e-6 ;1e-6 ;1e-6;1e-6 ;1e-6 ;1e-6]);
Torq=[1;0;0]*10^-5;%Input Torque
I=7/12*[20*2+30^2 0 0;0 2*20^2 0; 0 0 20*2+30^2]*10^-4;
Iin=inv(I);
wo=0.0012;% orbital angular velocity of satellite 2pi/(90*60)
T_q=(1/sqrt(30))*[1 2 3 4];%Target quaternion expressed as from orbit frame to target frame
%T_q=[0 0 0 1];
Rot=Quaternion_to_DCM(T_q);% Rotation Matrix from Orbit frame to Target frame
R=Rot;
I1=I(1,1);
I2=I(2,2);
I3=I(3,3);

% Appropriately Linearized matrices % Refer to documentation for further
% info
Astar1=-(wo)*[I2*R(3,2)-I3*R(2,2);...
    I3*R(1,2)-I1*R(3,2);...
    I1*R(2,2)-I2*R(1,2)];

Bstar=zeros(3,3);
Bstar(:,1)=[I2*R(2,2)^2+I3*R(3,2)^2;...
    -I1*R(1,2)*R(2,2);...
    -I1*R(1,2)*R(3,2)];
Bstar(:,2)=[-I2*R(1,2)*R(2,2);...
    I1*R(1,2)^2+I3*R(3,2)^2;...
    -I2*R(3,2)*R(2,2)];
Bstar(:,3)=[-I3*R(3,2)*R(1,2);...
    -I3*R(3,2)*R(2,2);...
    I1*R(1,2)^2+I2*R(2,2)^2];
Bstar=-(2*wo^2)*(Bstar);

Cstar=wo*[I2*R(2,2) -I3*R(3,2) 0;...
    I3*R(3,2) 0 -I1*R(1,2);...
    -I2*R(2,2) I1*R(1,2) 0];

Astar2=-(wo^2)*[I2*R(3,2)*R(2,2)-I3*R(2,2)*R(3,2);...
    I3*R(1,2)*R(3,2)-I1*R(3,2)*R(1,2);...
    I1*R(1,2)*R(2,2)-I2*R(1,2)*R(2,2)];

Estar=zeros(3,3);
Estar(:,1)=[I3*R(2,2)^2+I2*R(3,2)^2;...
    -I3*R(1,2)*R(3,2);...
    -I2*R(1,2)*R(3,2)];

Estar(:,2)=[-I3*R(1,2)*R(2,2);...
    I3*R(1,2)^2+I1*R(3,2)^2;...
    -I1*R(2,2)*R(3,2)];


Estar(:,3)=[-I2*R(1,2)*R(3,2);...
    -I1*R(2,2)*R(3,2);...
    I2*R(1,2)^2+I1*R(2,2)^2];

Estar=(2*wo^2)*(Estar);

Astar1_mod=Iin*Astar1;
Astar2_mod=Iin*Astar2;
Bstar_mod=Iin*Bstar;
Cstar_mod=Iin*Cstar;
Estar_mod=Iin*Estar;

Astar1_mod_33=[Astar1_mod(1) 0 0;...
    0 Astar1_mod(2) 0;...
    0 0 Astar1_mod(3)];
Dstar=-(wo)*skew_sym([R(1,2) R(2,2) R(3,2)]);
A_temp1=double((Astar1_mod_33+Cstar_mod-Dstar));
A_temp2=double(cat(2,A_temp1,Bstar_mod+Estar));
A_temp3=cat(2,0.5*eye(3),zeros(3,3));

%Linearized Model
A=cat(1,A_temp2,A_temp3);
%A=rand(6,6);
B_temp1=cat(2,Iin,zeros(3,3));
B=cat(1,B_temp1,zeros(3,6));
C=eye(6);
D=zeros(6,6);
u=cat(1,Torq,zeros(3,1));
t=0:0.1:10;
% sys=ss(A,(10^-4)*B,C,D)
% [Y,T,X]=step(sys,t)
% % x1=[1 0 0 0 0 0]*X';
% plot(t,Y(:,:,1))

%Solving for both linear and non linear models
[T,Z]=ode45(@solve_for_state_error_linear,0:0.1:700,[0.000;0.000;0.000;0;0;0],options);

[T,Y]=ode45(@solve_for_state_error_nonlinear,0:0.1:700,[0.000;0.000;0.000;0;0;0],options);
hold on;


omega_error=Y(:,1:3)-Z(:,1:3);
subplot(4,1,3)
plot(T,omega_error);
title('omega error in off-nadir pointing');

%solving for quaternion error

qnln4=1-sqrt((dot(Y(:,4:6)',Y(:,4:6)'))')
qln4=1-sqrt((dot(Z(:,4:6)',Z(:,4:6)'))');
qnln=cat(2,Y(:,4:6),qnln4);
qln=cat(2,Z(:,4:6),qln4);
% plot(T,qln)

%qerror=inv(qln)*qnln
 qlninv=cat(2,-Z(:,4:6),qln4);
 qm=@quaternion_multiply;
 s=size(T);
 qerror=zeros(s(1),4);
 for i=1:s(1)
     qerror(i,:)=qm(qlninv(i,:),qnln(i,:));
 end
 subplot(4,1,4);
 plot(T,qerror);
 title('quaternion error in off-nadir pointing');

%Non Linearized Model
function zprime=solve_for_state_error_nonlinear(t,z)
Rrc=@Quaternion_to_DCM;
Rrc([z(4:6,1);1])
zprime(1:3,1)= -Iin*cross((z(1:3,1)+wo*Rrc([z(4:6,1);1])*R*[0;1;0]),I*(z(1:3,1)+wo*Rrc([z(4:6,1);1])*R*[0;1;0]))+I\(Torq) -Astar2_mod;
% zprime(1:3,1)= cross(z(1:3,1),(I*(z(1:3,1))))+I\(Torq);
zprime(4:6,1)=0.5*(z(1:3,1))-0.5*cross(z(1:3,1),z(4:6,1));

% 
% zprime(1:3,1)= I*z(1:3,1)+I\(Torq);
% zprime(4:6,1)=0.5*(z(1:3,1));
end

function xprime=solve_for_state_error_linear(t,x)
xprime=A*x+B*u;
x;
end
end



