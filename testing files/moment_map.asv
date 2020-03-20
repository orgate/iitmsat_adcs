function M=moment_map(P,Q)
%P corresponds to the field associated constant in input matrix
%Q corresponds to the gain matrix
Z=coeff_mat_4_map_mom(P)
Q_int=Q(1:3,:)
Q=tocol(Q_int)
M_interm=linsolve(Z,Q)
M=col_to_mat(M_interm)
end