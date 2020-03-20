function B_field=magnetic_field_generator
%simulation parameters
t_sim=60000; %seconds
time_step=1; %seconds

%satellite Parameters
LatExptOff=45;  %latitude outside which the HEPD experiment is off. 
                %We need a positive value here. Switch-off latitudes are assumed
                %to be symmetric about the equator.

%orbit parameters
altitude=800000;
Omega=257.65*pi/180; %RAAN
omega=0; %argument of perigee. Has no significance for a circular orbit. 
%If an elliptical orbit is required, modifications must be made to the
%function ECIOrbitModel.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all
num=t_sim/time_step;
peak_torque=zeros(1,10);
oo=1;
for oo=1:1 %varying inclination
    inclination=(100-10*oo)*pi/180;
    dip=zeros(1,num);
    TA=zeros(3,3,num);
    TargetQuat=zeros(4,num);
    lat=zeros(1,num);
    long=zeros(1,num);
    r_ECEF=zeros(3,num);
    
    % Determining the target attitude in DCM & Quaternions
    for t=1:num
        r_ECI=ECIOrbitModel(time_step*t,Omega,inclination,omega,altitude);
        r_ECEF(:,t)=DCMECItoECEF(time_step*t)*r_ECI;
        [lat(t) long(t)]=LatLong(r_ECEF(:,t));
       
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
%         dip(t)=atand(-B_ECEF_sph(3)/sqrt(B_ECEF_sph(2)^2+B_ECEF_sph(1)^2));
%         TA(:,:,t)=TargetAttitude([B_ECEF_sph(1);B_ECEF_sph(2);-B_ECEF_sph(3)]);
        B_field(t,:)=B_ECEF_sph';
    end
 
end

   
    
   
        

        