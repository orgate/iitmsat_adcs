sum=zeros(5,9);
a=[2.7;2.7;6.7;8.5;8.5;700;4;10;6.7;];
b=[5;1;1;1;0.33;];
err2=zeros(5,9);
%l=1;
for i=1:9
    
MMerr = a(i)*0.25/10000;
%m=1;
for j=1:5
    SSerr = sin(b(j)*pi/180)/sqrt(3);
vres1 = [MMerr MMerr MMerr];
vres2 = [SSerr SSerr SSerr];
wres1 = [0 0 0];
wres2 = [0 0 0];
ang = (pi/6);
err = CombineVectorErrors(vres1,vres2,wres1,wres2,ang);
err2(j,i)=CombineVectorErrors(MMerr,SSerr,0,0,ang);
sum(j,i)=err(1);
%m=m+1;
end
%l=l+1;
end