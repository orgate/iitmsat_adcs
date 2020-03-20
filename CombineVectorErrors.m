function ydeg = CombineVectorErrors(vres1, vres2, wres1, wres2, aa)
%vres1 : error of first vector measurement in body coordinates
%vres2 : error of second vector measurement in body coordinates
%wres1 : error of first vector measurement in fixed coordinates
%wres2 : error of second vector measurement in fixed coordinates
%aa : angle between first and second vector

w1=[1;0;0];
w1=w1/sqrt(sum(w1.*w1));
v1=w1;
%aa=pi/6;
w2=[cos(aa);sin(aa);0];
w2=w2/sqrt(sum(w2.*w2));
v2=w2;

dw1 = [0;0;1]*sqrt(3)*wres1;
dw2 = -[0;0;1]*sqrt(3)*wres2;

dv1 = -[0;0;1]*sqrt(3)*vres1;
dv2 = [0;0;1]*sqrt(3)*vres2;

M1=skew_sym((w1)); M2=skew_sym((w2));
AA2=2*(M1')*M1+2*(M2')*M2;
     
b1=2*(M1*(dv1-dw1)+M2*(dv2-dw2));
tt=AA2\b1;
y=sqrt(sum(tt.*tt));
ydeg=y*180/pi;

end

     