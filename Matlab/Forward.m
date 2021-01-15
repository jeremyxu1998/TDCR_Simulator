function T = Forward(q, param, numSeg)
% Compute a forward pass for a multiple segment tendon robot

r = param(1);
L = param(2:end);
Q = reshape(q, 3, []);

T = eye(4);

for i = 1:numSeg
    T = T*forwardSection(Q(:,i), r, L(i));
end

end

function T = forwardSection(qi, r, l_i)

q_1 = qi(1);
q_2 = qi(2);
d_l = qi(3);

q_3 = -(q_1 + q_2);
l = l_i + d_l;

if q_1 ~= 0
    phi = atan2(0.5*q_1 + q_2, (-sqrt(3)/2)*q_1);
    kappa = (-q_1) / (l * r * cos(phi));
elseif q_2 ~=0
    phi = atan2(0.5*q_2 + q_3, (-sqrt(3)/2)*q_2) - 2*pi/3;
    kappa = (-q_2) / (l * r * cos(phi + 2*pi/3));
else
    T = eye(4);
    T(3,4) = l;
    return
end
theta = kappa * l;

% Forward kinematics
rotTrans = [cos(phi), -sin(phi), 0, 0;
            sin(phi), cos(phi), 0, 0;
            0, 0, 1, 0;
            0, 0, 0, 1];
bendTrans = [cos(theta), 0, sin(theta), (1-cos(theta))/kappa;
             0, 1, 0, 0;
             -sin(theta), 0, cos(theta), sin(theta)/kappa;
             0, 0, 0 ,1];
rotBackTrans = [cos(-phi), -sin(-phi), 0, 0;
                sin(-phi), cos(-phi), 0, 0;
                0, 0, 1, 0;
                0, 0, 0, 1];
T = rotTrans * bendTrans * rotBackTrans;

end
