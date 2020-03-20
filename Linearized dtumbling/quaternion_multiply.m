function y=quaternion_multiply(x1, x2)
y=sym(zeros(4,1));
y(1)=2*x1(1)*x2(1)-sum(x1.*x2);
temp=skew_sym([x1(2);x1(3);x1(4)])*[x2(2);x2(3);x2(4)];
y=[y(1);temp];
end