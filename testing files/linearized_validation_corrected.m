function T_F=linearized_validation_corrected
% This is the supporting function to validate the linearization of
% satellite equations
options = odeset('RelTol',1e-7,'AbsTol',(10^-1)*[1e-6 ;1e-6 ;1e-6;1e-6 ;1e-6 ;1e-6]);
options1 = odeset('RelTol',1e-7,'AbsTol',(10^-1)*[1e-6 ;1e-6 ;1e-6;1e-6 ;1e-6 ;1e-6;1e-6]);
Torq=[4;0;0]*10^-6;%Input Torque
I=7/12*[20*2+30^2 0 0;0 2*20^2+30^2  0; 0 0 2*20^2]*10^-4;
Iin=inv(I);
wo=0.0044;% orbital angular velocity of satellite 2pi/(90*60)
%T_q=(1/sqrt(30))*[1 2 3 4];%Target quaternion expressed as from orbit frame to target frame
T_q=[0 0 0 1];
Rot=Quaternion_to_DCM(T_q);% Rotation Matrix from Orbit frame to Target frame
R=Rot
I1=I(1,1);
I2=I(2,2);
I3=I(3,3);


% Appropriately Linearized matrices % Refer to documentation for further
% info

%==========Linearized Matrices for coupling term==============%
Astar1=+(wo)*[0 -I2*R(3,2) +I3*R(2,2);...
    I1*R(3,2) 0 -I3*R(1,2);...
    -I1*R(2,2) +I2*R(1,2) 0];
Astar2=(wo^2)*[I2*R(3,2)*R(2,2)-I3*R(2,2)*R(3,2);...
    I3*R(1,2)*R(3,2)-I1*R(3,2)*R(1,2);...
    I1*R(1,2)*R(2,2)-I2*R(1,2)*R(2,2)];
Bstar=zeros(3,3);
Bstar(:,1)=[-I2*R(2,2)^2-I3*R(3,2)^2;...
    I1*R(1,2)*R(2,2);...
    I1*R(1,2)*R(3,2)];
Bstar(:,2)=[I2*R(1,2)*R(2,2);...
    -I1*R(1,2)^2-I3*R(3,2)^2;...
    I2*R(3,2)*R(2,2)];
Bstar(:,3)=[I3*R(3,2)*R(1,2);...
    I3*R(3,2)*R(2,2);...
    -I1*R(1,2)^2+I2*R(2,2)^2];
Bstar=(2*wo^2)*(Bstar);
Cstar=wo*[0  I3*R(3,2) -I2*R(2,2);...
    -I3*R(3,2) 0 I1*R(1,2);...
    I2*R(2,2) -I1*R(1,2) 0];
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

Dstar=(wo)*skew_sym([R(1,2) R(2,2) R(3,2)]);%Matrix corresponding to LHS of Euler Equations

%=============Lineaized Gravity Gradient Torque===========================%
GG_const=-3*wo^2*[(I2-I3)*R(3,3)*R(2,3);...
    (I3-I1)*R(3,3)*R(1,3);...
    (I1-I2)*R(1,3)*R(2,3)];
GGstar(1,:)=2*(I2-I3)*[(R(3,3)^2-R(2,3)^2) R(2,3)*R(3,3) -R(1,3)*R(2,3)];
GGstar(2,:)=2*(I3-I1)*[-R(1,3)*R(2,3) (R(1,3)^2-R(3,3)^2) R(2,3)*R(3,3)];
GGstar(3,:)=2*(I1-I2)*[R(1,3)*R(3,3) -R(2,3)*R(3,3) (-R(1,3)^2+R(2,3)^2)];
GGstar=-(3*wo^2)*(GGstar);
GGstar_mod=Iin*GGstar;
%=========================================================================%


A_temp1=double((Astar1_mod+Cstar_mod+Dstar));
A_temp2=double(cat(2,A_temp1,Bstar_mod+Estar_mod));
A_temp3=cat(2,0.5*eye(3),zeros(3,3));

%Linearized Model
A=cat(1,A_temp2,A_temp3);
B_temp1=cat(2,Iin,zeros(3,3));
B=cat(1,B_temp1,zeros(3,6));
C=eye(6);
D=zeros(6,6);
u=cat(1,Torq,zeros(3,1));

%Solving for both linear and non linear models
[T,Z]=ode45(@solve_for_state_error_linear,0:0.1:100,[0.000;0.000;0.000;0;0;0],options);%%omega states are c(omega)cr and attitude states are c(q)r

[T,Y]=ode45(@solve_for_state_error_nonlinear,0:0.1:100,[0.000;0.000;0.000;0;0;0;1],options1);


omega_error=Y(:,1:3)-Z(:,1:3);
subplot(5,1,3)
plot(T,omega_error);
title('omega error in off-nadir pointing');


%solving for quaternion error

% qnln4=1-sqrt((dot(Y(:,4:6)',Y(:,4:6)'))');
qnln4=Y(:,7);
qln4=1-sqrt((dot(Z(:,4:6)',Z(:,4:6)'))');
% qnln=cat(2,Y(:,4:6),qnln4);
qnln=cat(2,Y(:,4:6),qnln4);
qln=cat(2,Z(:,4:6),qln4);
% plot(T,qln)

%qerror=inv(qln)*qnln
 qlninv=cat(2,-Z(:,4:6),qln4);
 qm=@quaternion_multiply;
 s=size(T);
 qerror=zeros(s(1),4);
 for i=1:s(1)
     qerror(i,:)=qm(qlninv(i,:)',qnln(i,:)');
 end
 subplot(5,1,5);
 plot(T,qerror);
 title('quaternion error in off-nadir pointing');
 subplot(5,1,4);
 plot(T,omega_error./Z(:,1:3));
 title('relative omega error in off-nadir pointing');
 subplot(5,1,1);
 plot(T,Z(:,1:3));
 title(' omega for linearized case in off-nadir pointing');
 subplot(5,1,2);
 plot(T,Y(:,1:3));
 title(' omega for non-linearized case in off-nadir pointing');
 
 figure, plot(T,Y(:,4:7));
  figure, plot(T,qln);
 
 
%Non Linearized Model
function zprime=solve_for_state_error_nonlinear(t,z)
Rrc=@Quaternion_to_DCM;%Rotation matrix from refence frame to control frame
qm=@quaternion_multiply;
t

ez(1:3,1)=z(1:3,1);
ez(4:7,1)=qm([-z(4:6,1);z(7,1)],T_q');%Error

wx = z(1,1);            % angular velocity along x 
wy = z(2,1);            % angular velocity along y 
wz = z(3,1);            % angular velocity along z 
z(4:7,1)=z(4:7,1)/norm(z(4:7,1));

%=======Kinematics Calculations=============%
%...Skew-symmetric matrix of angular velocities: 
skw_omeg = [  0   wz  -wy   wx 
         -wz    0   wx   wy 
          wy  -wx    0   wz 
         -wx  -wy  -wz    0];
 zprime(4:7,1)=0.5*skw_omeg*z(4:7,1);  
%========dynamics calculations==============%     
cRr=Rrc(z(4:7,1)); %Rotation from reference frame to control frame
L1_LHS=[z(4,1) -z(5,1) -z(6,1) z(7,1);...
    z(5,1) z(4,1) -z(7,1) -z(6,1);...
    z(6,1) z(7,1) z(4,1) z(5,1)]*zprime(4:7,1);
L2_LHS=[z(5,1) z(4,1) z(7,1) z(6,1);...
    -z(4,1) z(5,1) -z(6,1) z(7,1);...
    z(7,1) z(6,1) z(5,1) z(4,1)]*zprime(4:7,1);
L3_LHS=[z(6,1) -z(7,1) z(4,1) -z(5,1);...
    z(7,1) z(6,1) z(5,1) z(4,1);...
    -z(4,1) -z(5,1) z(6,1) z(7,1)]*zprime(4:7,1);

zprime(1:3,1)= -Iin*cross((z(1:3,1)-wo*cRr*R*[0;1;0]),I*(z(1:3,1)-wo*cRr*R*[0;1;0]))+I\(Torq)-Astar2_mod...
     +wo*cat(2,L1_LHS,L2_LHS,L3_LHS)*R*[0 1 0]';

end


function xprime=solve_for_state_error_linear(t,x)
xprime=A*x+B*u;
x;
end
end





