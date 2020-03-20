function rnk=controllability_chck(A,B)
% A is the state matrix and B is the Input matrix
a=size(A);
C=B;
for i=1:a(2)-1
C=cat(2,C,(A^i)*B);
end
rnk=rank(C);
return 