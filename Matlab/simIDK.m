%%% SIMULATE JACOBIAN CONTROL FOR SINGLE SEGMENT %%%

% Robot Parameters
r = 5e-3;
l_0 = 0.1;
param = [r; l_0];

% Initial Configuration
q_1_i = 0;
q_2_i = 0;
d_l_i = 0;
q_i = [q_1_i; q_2_i; d_l_i];
T_init = Forward(q_i, param);

% Target Configuration
q_1_t = 5e-5;
q_2_t = -2e-5;
d_l_t = 5e-3;
q_t = [q_1_t; q_2_t; d_l_t];
T_target = Forward(q_t, param);

% Sim Parameters
num_step = 1e4;
q_epsilon = 1e-5;
P_gain = 1e-3;

% Sim Variables
T_cur = T_init;
q_cur = q_i;
X = double.empty(6,0);
Q = double.empty(3,0);

% warning('off', 'all')

for step = 1:num_step
    
    Jb = estimateJacobian(q_cur, param, q_epsilon);
    Jb = Jb(4:6,:);
    % Jb_pseudo = (Jb'*Jb)\Jb'; % NOTE: Formula for a 'wide' Jacobian is different
    
    T_bd = invT(T_cur)*T_target;
    [S, theta] = tfLogMap(T_bd);
    w = [S(2,1), S(1,3), S(3,2)]';
    v = S(1:3,4);
    
    Vb = [w; v]; % This should be normalized: multiply by theta to get scaled version
    Vbv = Vb(4:6);
    
    q_new = q_cur + P_gain*theta*(Jb\Vbv);
    T_cur = Forward(q_new, param);
    q_cur = q_new;
    Q = [Q q_cur];
    
    % Store incremental target errors
    errR1 = norm(T_target(1:3,1) - T_cur(1:3,1));
    errR2 = norm(T_target(1:3,2) - T_cur(1:3,2));
    errR3 = norm(T_target(1:3,3) - T_cur(1:3,3));
    errX = T_target(1,4) - T_cur(1,4);
    errY = T_target(2,4) - T_cur(2,4);
    errZ = T_target(3,4) - T_cur(3,4);
    x = [errR1, errR2, errR3, errX, errY, errZ]';
    X = [X x];
    
    % Print intermediate reuslts
    if mod(step, 1e3) == 0
        T_cur
    end
    
end

% Plotting
t = linspace(0, num_step-1, num_step)';

% Orientation (rotation matrix norms)
figure(1);
tiledlayout(3,1)

ax1 = nexttile;
plot(ax1,t,X(1,:)')
ylabel(ax1,'Rx')

ax2 = nexttile;
plot(ax2,t,X(2,:)')
ylabel(ax2,'Ry')

ax3 = nexttile;
plot(ax3,t,X(3,:)')
ylabel(ax3,'Rz')

% Position
figure(2);
tiledlayout(3,1)

ax1 = nexttile;
plot(ax1,t,X(4,:)')
ylabel(ax1,'x')

ax2 = nexttile;
plot(ax2,t,X(5,:)')
ylabel(ax2,'y')

ax3 = nexttile;
plot(ax3,t,X(6,:)')
ylabel(ax3,'z')

% Configuration
figure(3);
tiledlayout(3,1)

ax1 = nexttile;
plot(ax1,t,Q(1,:)')
ylabel(ax1,'q_1')

ax2 = nexttile;
plot(ax2,t,Q(2,:)')
ylabel(ax2,'q_2')

ax3 = nexttile;
plot(ax3,t,Q(3,:)')
ylabel(ax3,'d')
