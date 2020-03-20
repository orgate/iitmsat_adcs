function r_ECI=ECIOrbitModel(t)
%t is the time in seconds since the satellite last passed through the
%ascending node
%Output is [X;Y;Z] in ECI
altitude=800000;
Omega=257.65*pi/180;
inclination=98.6*pi/180;
omega=0;
omega0=2*pi/(6000);
r_ECI=(6400000+altitude)*(RotMatrix(3,Omega))'*(RotMatrix(1,inclination))'*(RotMatrix(3,omega))'*[cos(omega0*t);sin(omega0*t);0];
end