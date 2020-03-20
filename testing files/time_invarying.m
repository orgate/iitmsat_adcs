function [T,Y]=main_nominal_time_varying_control%simulation parameters
t_sim=6000; %seconds
time_step=1000; %seconds

%satellite Parameters
LatExptOff=45;  %latitude outside which the HEPD experiment is off. 
                %We need a positive value here. Switch-off latitudes are assumed
                %to be symmetric about the equator.
global Isat Isatinv
Isat=7/12*[20*2+30^2 0 0;0 2*20^2 0; 0 0 20*2+30^2]*10^-4;

Isatinv=inv(Isat);
%orbit parameters
%altitude=800000;
%Omega=257.65*pi/180; %RAAN
%omega=0; %argument of perigee. Has no significance for a circular orbit. 
%If an elliptical orbit is required, modifications must be made to the
%function ECIOrbitModel.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all
%num=t_sim/time_step;
num=30;
peak_torque=zeros(1,10);
oo=1;
for oo=1:1 %varying inclination
%    inclination=(100-10*oo)*pi/180;
     dip=zeros(1,num);
     TA=zeros(3,3,num);
     TargetQuat=zeros(4,num);
     lat=zeros(1,num);
     long=zeros(1,num);
     r_ECEF=zeros(3,num);
     [time r_ECEF lat long]=ECIOrbitModel();
    % Determining the target attitude in DCM & Quaternions
     for t=1:num
%%       r_ECI=ECIOrbitModel(time_step*t,Omega,inclination,omega,altitude);
%         r_ECEF(:,t)=DCMECItoECEF(time_step*t)*r_ECI(:,t);
%         [lat(t) long(t)]=LatLong(r_ECEF(:,t));

        %initializing IGRF
        global gh
        if exist('GHcoefficients','file')==2
            load('GHcoefficients')
        else
            gh=GetIGRF11_Coefficients(1);
        end
        
        %Finding B, dip, and target attitude (here, target attitude assumes
        %the Cones-Not-Intersecting case.)
        B_ECEF_sph=igrf11syn(2011,800,lat(t),long(t));
        dip(t)=atand(-B_ECEF_sph(3)/sqrt(B_ECEF_sph(2)^2+B_ECEF_sph(1)^2));
        TA(:,:,t)=TargetAttitude([B_ECEF_sph(1);B_ECEF_sph(2);-B_ECEF_sph(3)]);
        TargetQuat(:,t)=DCM_to_Quaternion(TA(:,:,t));
    end
    %Converting to ECI reference frame. 
    TA_inertial=zeros(3,3,num);
    
    for t=1:num
        temp1=DCMSphToCart(r_ECEF(:,t));
        temp2=DCMECItoECEF(time_step*t);
        TA_inertial(:,:,t)=temp2*temp1'*TA(:,:,t)*temp1*temp2';
    end

    %Finding omega wrt body-fixed coordinates
    omega=zeros(3,num-1);
    for t=1:num-1
        TAdot=TA_inertial(:,:,t+1)-TA_inertial(:,:,t); 
        temp=TA_inertial(:,:,t)*TAdot'; %(R'*Rdot)
        omega(:,t)=[temp(2,3); temp(3,1);temp(1,2)];
    end
    
    %Target states generation
    
     so=Targetstates(omega,TargetQuat,num);
    % Solving for states at different time steps
    for n=1:num-1
    
     xo=so(1:6,n);
     % Required pole definition
     poles=[-0.50000 + 0.04161i ,-0.50000 - 0.04161i,-0.96 , -0.95 , -0.81 ,-0.91];
         %Calculation of state, input and gain matrices 
     
     A=@state_matrix;
     gain=@gain_matrix;
%      B=cat(2,Isatinv,zeros(3,3));
%      G=cat(1,B,zeros(3,6));
     F=A(xo(1:3));
     K=gain(xo(1:3),poles);
        
     [T,Y]=ode45(@solve_for_state_error,2:1:100,[0.005,0.004,0.002,0.1,0.2,0.3]);
     
     
    end

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
