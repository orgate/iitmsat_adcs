function [q]=DCM_to_Quaternion(R)
%quaternion=[vector scalar]'
q=zeros(4,1);
q(4,1)=0.5*sqrt(1+trace(R));
q(1:3,1)=(0.25/q(4))*[R(2,3)-R(3,2);R(3,1)-R(1,3);R(1,2)-R(2,1)]';
end 