function Power=CalcFlux(S_body)
%Here, DCM is from ECEF to Body-Coords
satellite_dims=[.2;.2;.3]
CoverageEfficiency=[1 1; 1 1; 1 1] %[+roll -roll; +pitch -pitch; +yaw -yaw]
areas=[satellite_dims(2)*satellite_dims(3);satellite_dims(1)*satellite_dims(3);satellite_dims(1)*satellite_dims(2)];
eta=zeros(3,1);
for ii=1:3
    if (S_body(ii)>0) eta(ii)=CoverageEfficiency(ii,1);
    else
        eta(ii)=CoverageEfficiency(ii,2);
    end
end
eta
Power=sum((eta.*S_body).*areas)
end