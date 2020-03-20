function r_ECI=ECIOrbitModel(t,Omega,inclination,omega,altitude)
%t is the time in seconds since the satellite last passed through the
%ascending node
%Output is [X;Y;Z] in ECI
omega0=2*pi/(6000);
r_ECI=(6400000+altitude)*(RotMatrix(3,Omega))'*(RotMatrix(1,inclination))'*[cos(omega0*t);sin(omega0*t);0];
end