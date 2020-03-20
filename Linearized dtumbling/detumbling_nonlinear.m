function X_dot=detumbling_nonlinear(X,b,bt,w0,I,K)

q=[X(1); X(2); X(3); X(4)];
w=[X(5);X(6);X(7)];

%Define Sigmas from I in principal form.
s1=(I(2,2)-I(3,3))/I(1,1);
s2=(I(3,3)-I(1,1))/I(2,2);
s3=(I(1,1)-I(2,2))/I(3,3);

%Calculate q_dot from q and P
v=[w;0]-quaternion_multiply(quaternion_multiply(quaternion_inverse(q),[0;w0;0;0]),q);
P=0.5*[0 v(3) -v(2) v(1);-v(3) 0 v(1) v(2);v(2) -v(1) 0 v(3);-v(1) -v(2) -v(3) 0];
q_dot=P*q;

%Calculate w_dot **B (earth's Magnetic field) is in Orbit coordinate System(OCS)**
Y=[skew_sym(bt)*b;0];
v=quaternion_multiply(quaternion_multiply(quaternion_inverse(q),[0; b]),q);
S=[0 v(3) -v(2);-v(3) 0 v(1);v(2) -v(1) 0];
temp=quaternion_multiply(quaternion_multiply(quaternion_inverse(q),Y),q);
w_dot=(K*([temp(1);temp(2);temp(3)]-((S^2)*w)))+[s1*w(2)*w(3);s2*w(3)*w(1);s3*w(1)*w(2)];

%calculate X_dot
X_dot=[q_dot;w_dot];
end
