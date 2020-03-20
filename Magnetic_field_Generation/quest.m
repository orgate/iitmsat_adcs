syms t1 t2 t3
v=[1; 5; 1];
w=[1;1;1];
t=[t1;t2;t3]
dv=[.1; .3; .4];
dw=[.1; .11; .1];
A=[1 0 0;0 cosd(45) -sind(45);...
    0 sind(45) cosd(45)];
l=dv-skew_sym(t)*A*w-A*dw
loss=sum(l.*l)
eq=sym(zeros(3,1))
for ii=1:3
    eq(ii)=diff(loss,t(ii))
end
% AA=sym(eye(3));
for ii=1:3
    for jj=1:3
        temp=coeffs(eq(ii),t(jj));
        AA(ii,jj)=temp(2);
       
    end
end
bb=zeros(3,1)
for ii=1:3
    temp=coeffs(eq(ii))
    bb(ii)=temp(1)
end

t=AA\bb
%cat(1,AA(1),AA(2),AA(3));