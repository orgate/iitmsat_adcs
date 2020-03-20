function yprime=control(t,y)
a=@testfunc1;
yprime=[y(2);a(t,x)*sin(y(1))];