function [J]=coeff_mat_4_map_mom(W)

%this function converts the symmetric -(inv(I)/norm(B))*(BxB) into a 
%18x18 coefficient matrix to solve for M1. J*M=K
J=zeros(18,18);
j=1;
for i=1:6
        J(j,3*(i-1)+1:3*(i-1)+3)=W(1,:);
        j=j+1;
end


for i=1:6
        J(j,3*(i-1)+1:3*(i-1)+3)=W(2,:);
        j=j+1;
end

for i=1:6
        J(j,3*(i-1)+1:3*(i-1)+3)=W(3,:);
        j=j+1;
end

