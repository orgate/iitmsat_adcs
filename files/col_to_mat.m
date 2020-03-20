function [M]=col_to_mat(K)
M=zeros(3,6);
for i=1:6
    M(:,i)=K(3*(i-1)+1:3*(i-1)+3);
end