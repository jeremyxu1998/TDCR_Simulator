function [S, theta] = tfLogMap(T)
% Computes the logarithmic mapping of a transformation matrix

R = T(1:3,1:3);
p = T(1:3,4);
traceR = trace(R);
theta = acos((traceR - 1)/2); % angle of rotation
w_skew = (1/(2*sin(theta))) * (R - R');

V = eye(3) + (1-cos(theta))/(theta^2) * w_skew + (theta-sin(theta))/(theta^3) * (w_skew)^2;
v = inv(V)*p;

S = zeros(4,4);
S(1:3,1:3) = w_skew;
S(1:3,4) = v;

end

