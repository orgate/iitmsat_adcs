
l_x=2; l_y=3; l_z=4;
g=[-1 -1;-1 1;1 1;1 -1]'
f(:,:,1)=.5*[l_x*[1 1 1 1];[l_y 0; 0 l_z]*g]
f(:,:,2)=.5*[-l_x*[1 1 1 1];[l_y 0; 0 l_z]*g]

temp=[l_x 0; 0 l_z]*g
f(:,:,3)=.5*[temp(1,:);l_y*[1 1 1 1];temp(2,:)]
f(:,:,4)=.5*[temp(1,:);-l_y*[1 1 1 1];temp(2,:)]

f(:,:,5)=.5*[[l_x 0; 0 l_y]*g;l_z*[1 1 1 1]]
f(:,:,6)=.5*[[l_x 0; 0 l_y]*g;-l_z*[1 1 1 1]]

close all
figure
axis(3*[l_x*[-1 1] l_y*[-1 1] l_z*[-1 1]])
axis equal
set(gca,'CameraPosition',[l_x l_y 2*l_z])
set(gca,'CameraViewAngle',110)
set(gca,'CameraPosition',[l_x l_y 2*l_z])
hold on
x=[f(1,:,1);f(1,:,2);f(1,:,3);f(1,:,4);f(1,:,5);f(1,:,6)]'
y=[f(2,:,1);f(2,:,2);f(2,:,3);f(2,:,4);f(2,:,5);f(2,:,6)]'
z=[f(3,:,1);f(3,:,2);f(3,:,3);f(3,:,4);f(3,:,5);f(3,:,6)]'
c=(linspace(0,1,6)'*[1 1 1 1])'
hc=fill3(x,y,z,c)
omega=[.1 .2 .3]
R=eye(3);

for t=1:100
    R=R+double(skew_sym(omega*t/100))*R/100
    for ii=1:6
        f(:,:,ii)=R*f(:,:,ii)
        set(hc(ii),'vertices',f(:,:,ii)')
    end
    drawnow
end


