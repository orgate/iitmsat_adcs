function [lat long]=LatLong(r_ECEF)
lat=(180/pi)*atan2(r_ECEF(3),sqrt(r_ECEF(2)^2+r_ECEF(1)^2));%lat and long in degrees. lat from -89 to 90 and long from -180 to 179
long=(180/pi)*atan2(r_ECEF(2),r_ECEF(1));
end