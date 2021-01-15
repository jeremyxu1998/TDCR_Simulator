%%% SIMULATE JACOBIAN CONTROL FOR SINGLE SEGMENT %%%

% Robot Parameters
r = 5e-3;
l_1 = 0.1;
l_2 = 0.1;
l_3 = 0.1;
param = [r; l_1; l_2; l_3];
numSeg = 3;

% Initial Configuration
q1_1_i = 0;
q1_2_i = 0;
d1_l_i = 0;

q2_1_i = 0;
q2_2_i = 0;
d2_l_i = 0;

q3_1_i = 0;
q3_2_i = 0;
d3_l_i = 0;

if numSeg == 1
    q_i = [q1_1_i; q1_2_i; d1_l_i];
elseif numSeg == 2
    q_i = [q1_1_i; q1_2_i; d1_l_i; q2_1_i; q2_2_i; d2_l_i];
else
    q_i = [q1_1_i; q1_2_i; d1_l_i; q2_1_i; q2_2_i; d2_l_i; q3_1_i; q3_2_i; d3_l_i];
end
T_init = Forward(q_i, param, numSeg);
eulZYX_init = rotm2eul(T_init(1:3,1:3));

% Target Configuration
q1_1_t = 5e-5;
q1_2_t = -2e-5;
d1_l_t = 5e-3;

q2_1_t = 0;
q2_2_t = 0;
d2_l_t = 0;

q3_1_t = 0;
q3_2_t = 0;
d3_l_t = 0;

if numSeg == 1
    q_t = [q1_1_t; q1_2_t; d1_l_t];
elseif numSeg == 2
    q_t = [q1_1_t; q1_2_t; d1_l_t; q2_1_t; q2_2_t; d2_l_t];
else
    q_t = [q1_1_t; q1_2_t; d1_l_t; q2_1_t; q2_2_t; d2_l_t; q3_1_t; q3_2_t; d3_l_t];
end
T_target = Forward(q_t, param, numSeg);
eulZYX_target = rotm2eul(T_target(1:3,1:3));

%% Jacobian Control

% Sim Parameters
num_step = 1e4;
q_epsilon = 1e-5;
P_gain = 1e-3;
control_mode = 'full';

% Sim Variables
T_cur = T_init;
q_cur = q_i;
X = double.empty(6,0);
Q = double.empty(9,0);

% warning('off', 'all')

for step = 1:num_step
    
    Jb = estimateJacobian(q_cur, param, numSeg, q_epsilon);
    if strcmp(control_mode, 'orientation')
        Jb = Jb(1:3,:);
    elseif strcmp(control_mode, 'position')
        Jb = Jb(4:6,:);
    end
    
    if size(Jb, 1) == size(Jb, 2)
        Jb_pseudo = Jb\eye(size(Jb)); % Formula for square Jacobian inverse
    elseif size(Jb, 1) > size(Jb, 2)
        Jb_pseudo = (Jb'*Jb)\Jb'; % Formula for a 'tall' Jacobian pseudoinverse
    else
        Jb_pseudo = Jb'*((Jb*Jb')\eye(size(Jb,1))); % Formula for a 'wide' Jacobian pseudoinverse
    end
    
    T_bd = invT(T_cur)*T_target;
    [S, theta] = tfLogMap(T_bd);
    w = [S(3,2), S(1,3), S(2,1)]';
    v = S(1:3,4);
    
    Vb = [w; v]; % This should be normalized: multiply by theta to get scaled version
    Vbw = Vb(1:3);
    Vbv = Vb(4:6);
    
    % Control Step
    
    if strcmp(control_mode, 'orientation')
        q_new = q_cur + P_gain*theta*(Jb_pseudo*Vbw);
    elseif strcmp(control_mode, 'position')
        q_new = q_cur + P_gain*theta*(Jb_pseudo*Vbv);
    else
        q_new = q_cur + P_gain*theta*(Jb_pseudo*Vb);
    end
    
    T_cur = Forward(q_new, param, numSeg);
    q_cur = q_new;
    Q = [Q q_cur];
    
    % Store incremental target errors
    eulZYX_cur = rotm2eul(T_cur(1:3,1:3));
    
    errEulX = eulZYX_target(3) - eulZYX_cur(3);
    errEulY = eulZYX_target(2) - eulZYX_cur(2);
    errEulZ = eulZYX_target(1) - eulZYX_cur(1);
    errX = T_target(1,4) - T_cur(1,4);
    errY = T_target(2,4) - T_cur(2,4);
    errZ = T_target(3,4) - T_cur(3,4);
    x = [errEulX, errEulY, errEulZ, errX, errY, errZ]';
    X = [X x];
    
    % Print intermediate reuslts
    if mod(step, 1e3) == 0
        T_cur
    end
    
end

% Plotting
t = linspace(0, num_step-1, num_step)';

% Orientation (euler angle error)
figure(1);
tiledlayout(3,1)

ax1 = nexttile;
plot(ax1,t,X(1,:)')
ylabel(ax1,'EulX')

ax2 = nexttile;
plot(ax2,t,X(2,:)')
ylabel(ax2,'EulY')

ax3 = nexttile;
plot(ax3,t,X(3,:)')
ylabel(ax3,'EulZ')

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

%% Configuration plots

% Configuration (section 1)
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

if numSeg > 1
    % Configuration (section 2)
    figure(4);
    tiledlayout(3,1)

    ax1 = nexttile;
    plot(ax1,t,Q(4,:)')
    ylabel(ax1,'q_1')

    ax2 = nexttile;
    plot(ax2,t,Q(5,:)')
    ylabel(ax2,'q_2')

    ax3 = nexttile;
    plot(ax3,t,Q(6,:)')
    ylabel(ax3,'d')
end

if numSeg > 2
    % Configuration (section 3)
    figure(5);
    tiledlayout(3,1)

    ax1 = nexttile;
    plot(ax1,t,Q(7,:)')
    ylabel(ax1,'q_1')

    ax2 = nexttile;
    plot(ax2,t,Q(8,:)')
    ylabel(ax2,'q_2')

    ax3 = nexttile;
    plot(ax3,t,Q(9,:)')
    ylabel(ax3,'d')
end
