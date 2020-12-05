% % Symbolic variable def
% syms q_1 q_2 d_l;
% q_1 = sym('q_1', 'real'); % robot config
% q_2 = sym('q_2', 'real');
% d_l = sym('d_l', 'real');
% q = [q_1, q_2, d_l];
% r = sym('r', 'real');
% % syms r; % robot parameters
% l_0 = sym ("l_0", 'real');

% Robot parameters
r = 5e-3;
l_0 = 0.1;
num_seg = 1;
num_tend = 3;  % currently don't support 4 tendons
num_dof = num_seg * num_tend;

% Initial Configuration
q_1_i = 0;
q_2_i = 0;
d_l_i = 0;
q = [q_1_i, q_2_i, d_l_i];
% subs(q);
% T = Forward(q, r, l_0);
T_init = eye(4);
T_init(3,4) = l_0;

% Target Configuration
q_1_t = 5e-3;
q_2_t = -2e-3;
d_l_t = 5e-3;
q_t = [q_1_t, q_2_t, d_l_t];
% subs(q);
T_target = Forward(q_t, r, l_0);
T_target

% Loop setup
num_time_step = 1e5;
epsilon_t = 1e-5; % time step length
epsilon = 1e-5; % very small movement step
delta_T = (T_target - T_init) / num_time_step;

T_cur = T_init;
q_cur = q;
p_gain = 1;

warning('off','all')

for time_step = 1 : num_time_step
    
    % % Geometric Jacobian, classical approach
    % J_b = zeros(6:n);
    % for i = 1:n
    %     T_i_exp = eye(4);
    %     for j = n:-1:i+1
    %         skew_B_i = [0, -B_i(3), B_i(2); B_i(3) 0 -B_i(1); -B_i(2), B_i(1), 0];
    %         T_i_exp = T_i_exp * inv(exp(skew_B_i * twist_theta_i));
    %     end
    %     J_bi = adjoint(T_i_exp) * B_i;
    %     J_b(:,i) = J_bi;
    % end

    % Geometric Jacobian, numerical approximation
    J_b = zeros(6:num_dof);
    for i = 1 : num_dof
        q_di = q;
        q_di(i) = q_di(i) + epsilon;
        T_i = Forward(q_di, r, l_0);
        dT_dq = (T_i - T_cur) / epsilon;
        J_bi_skew = inv(T_cur) * dT_dq;
        J_bi = [J_bi_skew(3,2) J_bi_skew(1,3) J_bi_skew(2,1) J_bi_skew(1,4) J_bi_skew(2,4) J_bi_skew(3,4)]'; % [w, v]
        J_b(:,i) = J_bi;
    end

    % Pose
    % X = T * [0, 0, l_0, 1]'; % Tip position
    % % Analytical Jacobian
    % J_pos = [diff(X(1), q_1), diff(X(1), q_2), diff(X(1), d_l);
    %          diff(X(2), q_1), diff(X(2), q_2), diff(X(2), d_l);
    %          diff(X(3), q_1), diff(X(3), q_2), diff(X(3), d_l)];
    % J_rot

    T_next_desired = T_init + time_step * delta_T;
    
    J_b_pseudo = J_b' * inv(J_b*J_b');
%     twist_skew = inv(T_cur) * (T_next_desired - T_cur);  % use delta_T as T_dot
%     twist = [twist_skew(3,2) twist_skew(1,3) twist_skew(2,1) twist_skew(1,4) twist_skew(2,4) twist_skew(3,4)]';
    T_bd = inv(T_cur) * T_next_desired;
    % Logarithm map
    theta = acos((trace(T_bd(1:3,1:3))-1) / 2); % angle of rotation
    omega = (1/(2*sin(theta))) * [T_bd(3,2)-T_bd(2,3) T_bd(1,3)-T_bd(3,1) T_bd(2,1)-T_bd(1,2)]'; % ω, normalized axis
    omega_skew = [0 -omega(3) omega(2); omega(3) 0 -omega(1); -omega(2) omega(1) 0];
    V = eye(3) + (1-cos(theta))/(theta^2) * omega_skew + (theta-sin(theta))/(theta^3) * (omega_skew)^2;
    
    twist_v = inv(V) * T_bd(1:3, 4);
    twist = [omega; twist_v];
    q_dot = J_b_pseudo * twist;

    q_new = (q_cur' + q_dot * epsilon_t * p_gain)';
    T_cur = Forward(q_new, r, l_0);
    q_cur = q_new;
    
    % Print intermediate reuslts
    if mod(time_step, 1e4) == 0
        T_cur
    end

end
