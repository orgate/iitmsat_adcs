dip=zeros(1,num);
for t=1:(num)
        r_ECI=ECIOrbitModel(time_step*t,Omega,inclination,omega,altitude);
        r_ECEF(:,t)=DCMECItoECEF(time_step*t)*r_ECI;
        [lat(t) long(t)]=LatLong(r_ECEF(:,t));

         global gh
        if exist('GHcoefficients','file')==2
            load('GHcoefficients')
        else
            gh=GetIGRF11_Coefficients(1);
        end
        B_ECEF_sph=igrf11syn(2011,800,lat(t),long(t));
        dip(t)=atand(-B_ECEF_sph(3)/sqrt(B_ECEF_sph(2)^2+B_ECEF_sph(1)^2));
end
figure
hc=plot(1:(num-2),dip(1:(num-2)),1:(num-2),tt);
set(hc,'linewidth', 2);
