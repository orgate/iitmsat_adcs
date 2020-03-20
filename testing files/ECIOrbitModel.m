%function r_ECI=ECIOrbitModel(t,Omega,inclination,omega,altitude)
function [time r_ECEF lat long]=ECIOrbitModel(num, time_step)
%t is the time in seconds since the satellite last passed through the
%ascending node
%Output is [X;Y;Z] in ECI
%omega0=2*pi/(6000);

%num=6000;
r_ECEF=zeros(3,num);
lat=zeros(1,num);
long=zeros(1,num);

[tim take(1,:) take(2,:) take(3,:)]=testmat(2011,12,4,1,30,0,num,time_step);
r_ECI=[take(1,1:num); take(2,1:num); take(3,1:num)];
time=tim;
for t=1:num
    r_ECEF(:,t)=DCMECItoECEF(time_step*t)*r_ECI(:,t);
    [lat(t) long(t)]=LatLong(r_ECEF(:,t));
end
end