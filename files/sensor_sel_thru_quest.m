
%%%%%%
%The following m file assists in magnetometer and sunsensor selection by 
%finding the rotation error matrix. The rotation angle over eigen axis of 
%the rotation error matrix should not exceed 5 degrees when all the sensor 
%errors(sensor errors and errors from IGRF and orbital sun vector model)are
%considered

%%%%%%
%PROCEDURE: 
%1) Generate a random vector for mag field and sun sensor in orbit frame
%2) Generate a random vector for mag field and sun sensor in body frame
%3) Actual Rotation matrix is generated using quest algorithm
%4) Two cases a)with & with out error in orbit frame vectors may be
%considered
%5) Generate the errors in sensor detection based on the resolution issues
%6) Obtain the Rotation error matrix and check for rotation within 5
%degrees.
%%%%%%%
n=10000; % no of iterations
m=0;
Ted=zeros(n,1);
Mr=[4;10;50;750;1000];
SSr=[0.01;0.3;1;5];
sMr=size(Mr);
sSSr=size(SSr);
comparisons=zeros(sMr(1),sSSr(1),4);%4 corresponds to mean, std,sum of mean and std, overflows

for p=1:sMr(1)
    
        for q=1:sSSr(1)
            
for i=1:n
    
%Generation of random vectors for mag field and sun sensor in orbit & body frame

vo=40000*unifrnd(-1,1,[3,1]);%Actual Magnetic Field vector in orbit frame
von=vo/sqrt(sum(vo.*vo));%Normalized

vb=40000*unifrnd(-1,1,[3,1]);% Actual Magnetic Field vector in Body frame
vbn=vb/sqrt(sum(vb.*vb));

%For sunsensors(using simplistic vector generation model
temp_ang=60*unifrnd(-1,1,[3,1]);
wb=cosd(temp_ang);%Actual Sun vector in body frame
wbn=wb/sqrt(sum(wb.*wb));

wo=unifrnd(-1,1,[3,1]);% Actual Sun vector in orbit frame
won=wo/sqrt(sum(wo.*wo));



%%%%%Calculation of actual rotation matrix using Quest algorithm%%%%%%%%%%
K=zeros(4,4);
s=[1;1];%weights
B=s(1)*(vbn*von')+s(2)*(wbn*won');
S=B+B';
Z=[B(2,3)-B(3,2),B(3,1)-B(1,3),B(1,2)-B(2,1)]';
sig=trace(B);
K(1:3,1:3)=S-sig*eye(3);
K(4,1:3)=Z';
K(1:3,4)=Z;
K(4,4)=sig;
[E1, lambda]=eig(K);
[val1 posn1]=max(lambda);
[val2 posn2]=max(val1);
A=Quaternion_to_DCM(E1(:,posn2)); % Actual Rotation matrix

%%%%%%Calculation of Rotation matrix with error in sensor values%%%%%%%%%%

%%%%%%Case1: No errors in vectors in orbit frame%%%%%%%

%Generation of unifrndom vectors for magnetic field and sun sensor with
%resolution errors


dvb=unifrnd(-Mr(p),Mr(p),[3,1]);
%dvb=zeros(3,1);
vdb=vb+dvb;

%For sunsensors(using simplistic vector generation model
ang_res_err=unifrnd(-SSr(q),SSr(q),[3,1]);%resolution error
error_angle=temp_ang+ang_res_err;
wdb=cosd(error_angle);
%dwb=zeros(3,1);

dvo=[0;0;0];
vdo=vo+dvo;

dwo=[0;0;0];
wdo=wo+dwo;


vdbn=vdb/sqrt(sum(vdb.*vdb));%Normalized
wdbn=wdb/sqrt(sum(wdb.*wdb));
vdon=vdo/sqrt(sum(vdo.*vdo));
wdon=wdo/sqrt(sum(wdo.*wdo));

%%%%%%Calculation of rotation matrix with sensor errors using quest algo%%
Ke=zeros(4,4);
se=[1;1];%weights
Be=se(1)*(vdbn*vdon')+s(2)*(wdbn*wdon');
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
Ad=Quaternion_to_DCM(E2(:,posn2));% Rotation matrix with errors

% Calculation of error rotation matrix
Error_Rot=Ad*A';

% Calculation of rotation of error_rot over eigen axis
[C V]=eigs(Error_Rot);
te=abs(atan2(imag(V(2,2)),abs(real(V(2,2)))));%% theta error
ted=(180/pi)*te;
Ted(i,1)=ted;
if(Ted(i,1)>=2.5)
    m=m+1;
end
end
PD=fitdist(Ted,'normal')
comparisons(p,q,1)=mean(Ted);
comparisons(p,q,2)=std(Ted);
comparisons(p,q,3)=comparisons(p,q,1)+comparisons(p,q,2);
comparisons(p,q,4)=m;
m=0;
end
end





