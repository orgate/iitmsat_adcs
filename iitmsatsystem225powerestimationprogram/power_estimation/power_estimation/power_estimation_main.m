Beta=pi;
%n=1000;
for t=1:6000
    r_ECI=ECIOrbitModel(t);
    r_ECEF=DCMECItoECEF(t)*r_ECI
    S_ECI=NotEclipse(r_ECI,SolarIntensityVector(Beta))*SolarIntensityVector(Beta);
    testS_ECI(t,:)=S_ECI
   % S_ECI=[1;0;0]
    S_ECEF=DCMECItoECEF(t)*S_ECI
    S_ECEF_sph=DCMCartToSph(r_ECEF)*S_ECEF
    [lat long]=LatLong(r_ECEF);
    B_ECEF=igrf11syn(2011,800,lat,long)
    B_ECEF_sph=DCMCartToSph(r_ECEF)*B_ECEF
    DCM=TargetAttitude(B_ECEF_sph)
    S_body=DCM*S_ECEF_sph
    Power(t)=CalcFlux(S_body)
end

figure
hc1=plot(Power)
set(hc1, 'linewidth', 2)
title('Power vs. time','fontsize', 15, 'fontweight', 'bold');
grid on;
xlabel('Time, seconds')
ylabel('Solar Flux, watts')

figure
hc2=plot(testS_ECI)
set(hc2, 'linewidth', 2)
legend
title('Solar Intensity in ECI coordinates vs. time','fontsize', 15, 'fontweight', 'bold');
grid on;
xlabel('Time, seconds')
ylabel('Solar Intensity, watts/m^2')
