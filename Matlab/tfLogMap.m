function [S, theta] = tfLogMap(T)
% Computes the logarithmic mapping of a transformation matrix

R = T(1:3,1:3);
p = T(1:3,4);

if R == eye(3)
    w_skew = zeros(3);
    v = p/norm(p);
    theta = norm(p);
else
    traceR = trace(R);
    theta = acos((traceR - 1)/2); % angle of rotation
    w_skew = (1/(2*sin(theta))) * (R - R');

    G_inv = eye(3)/theta + 0.5*w_skew + (1/theta - 0.5*cot(theta/2)) * (w_skew)^2;
    v = G_inv*p;
end

S = zeros(4,4);
S(1:3,1:3) = w_skew;
S(1:3,4) = v;

end

