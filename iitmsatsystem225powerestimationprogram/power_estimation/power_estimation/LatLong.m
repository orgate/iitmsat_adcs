function [lat long]=LatLong(r_ECEF)
lat=atan2(r_ECEF(3),sqrt(r_ECEF(2)^2+r_ECEF(1)^2));
long=atan2(r_ECEF(2),r_ECEF(1));
end