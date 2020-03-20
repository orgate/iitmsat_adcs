function M = RotMatrix(axis, theta)
%axis = 1, 2 or 3
if (axis==1)
    M=[1 0 0; 0 cos(theta) sin(theta); 0 -sin(theta) cos(theta)];
end

if (axis==2)
    M=[cos(theta) 0 -sin(theta);0 1 0; sin(theta) 0 cos(theta)];
end

if (axis==3)
    M=[cos(theta) sin(theta) 0; -sin(theta) cos(theta) 0; 0 0 1];
end

end