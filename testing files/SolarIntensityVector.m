function S_ECI = SolarIntensityVector(Beta)
Omega=257.65*pi/180;
solar_constant=1367.7;
unit_S_ECI=[cos(pi*23.5/180)*cos(Beta+Omega); cos(pi*23.5/180)*sin(Beta+Omega); sin(pi*23.5/180)]
S_ECI=solar_constant*unit_S_ECI
end