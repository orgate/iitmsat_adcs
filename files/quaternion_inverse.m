function y=quaternion_inverse(x)
y=sym(zeros(4,1));
y(1)=x(1);
y(2)=-x(2);
y(3)=-x(3);
y(4)=-x(4);
end