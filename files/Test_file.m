function T_F=Test_file
% I=7/12*[20*2+30^2 0;0 2*20^2]*10^-4;
I=7/12*[20*2+30^2 0 0;0 2*20^2 0; 0 0 20*2+30^2]*10^-4;
[T,Y]=ode45(@test,0:0.1:100,[1; 2; 3; 5]);
plot(T,Y(:,4))

function [yprime]=test(t,y)
yprime(1:3,1)=cross(y(1:3,1),I*y(1:3,1));
% xprime=cross(y(4:6),I*y(4:6));
yprime(4)=-2*y(4,1);
end


end


