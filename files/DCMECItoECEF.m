function r_ECEF=DCMECItoECEF(t)
%t is the time in seconds since the Greenwich meridian last coincided with
%the vernal equinox.
%r_ECEF and r_ECI are in Cartesian form.
r_ECEF=RotMatrix(3,t*7.2722e-005) %7.2722e-005=2*pi/(24*3600)
end