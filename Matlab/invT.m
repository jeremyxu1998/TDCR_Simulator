function T_inv = invT(T)
% Get incremental transformation matrix

R = T(1:3,1:3);
d = T(1:3,4);
R_inv = R';
d_inv = -R_inv*d;

T_inv = eye(4);
T_inv(1:3,1:3) = R_inv;
T_inv(1:3,4) = d_inv;

end

