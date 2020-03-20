function y=skew_sym(x)
y=zeros(3,3);
y(1,2)=x(3); y(2,1)=-x(3);
y(1,3)=-x(2); y(3,1)=x(2);
y(2,3)=x(1); y(3,2)=-x(1);
end