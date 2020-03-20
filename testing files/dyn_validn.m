
function [T Y]=dyn_validn(sigma)
options = odeset('RelTol',1e-6,'AbsTol',[1e-6 ;1e-6 ;1e-6]);
M=[2;2;2]*10^-4;%Input Torque
Isat=7/12*[20*2+30^2 0 0;0 30^2 0; 0 0 30*2+30^2]*10^-4;
M1=inv(Isat)*M;
time_step=0.001;
sim_time=100;
%solving for omega
[T,Y]=ode45(@euler_equations,0:time_step:sim_time,[0.005;0.004;0.002],options);

function dy = euler_equations(t,y)

dy = zeros(3,1);    % a column vector
dy(1) = sigma(1)*y(2) * y(3)+M1(1);
dy(2) = sigma(2)*y(1) * y(3)+M1(2);
dy(3) = sigma(3)* y(1) * y(2)+M1(3);
end

plot(T,Y);
s=size(Y);
En_in_temp=zeros(s(1),1);
for i =1:3
 En_in_temp= En_in_temp+M(i)*Y(:,i);
end

Energy_input=sum(En_in_temp)*time_step;% integration of tau*omega*dt
Initial_kinetic_energy=0.5*dot(Isat*Y(1,:)',Y(1,:)');
Final_kinetic_energy=0.5*dot(Isat*Y(s(1),:)',Y(s(1),:)');

E_I=vpa(Energy_input)
I_K_E=vpa(Initial_kinetic_energy)
F_K_E=vpa(Final_kinetic_energy)



if(abs(((Final_kinetic_energy-Initial_kinetic_energy)-Energy_input))<10^-5)
    display('validated')
else
    display('not valid')
end


end