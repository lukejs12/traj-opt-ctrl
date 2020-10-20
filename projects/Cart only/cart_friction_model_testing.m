% Investigating the model described in 'A New Continuously Differentiable
% Friction Model' to model friction in the cart. 

mu = @(qd, gamma) gamma(1)*(tanh(gamma(2)*qd - tanh(gamma(3)*qd))) + ...
    gamma(4)*tanh(gamma(5)*qd) + gamma(6)*qd;

% v = -3:0.1:3;
% F_f = mu(v, [1, 1, 1, 1, 1, 1]);
% plot(v, F_f);
% grid on;

% Forcing function
u = @(t, x) 7*sin(2*pi.*t);

% % disp('Creating pendulum controller interface');
% % interface = PendulumController;

% Follow the trajectory with open loop controller
disp('OL controller');
T = 1;
x0 = [-0.3 0]';
[t_traj, x_traj, u_traj] = interface.followTrajectory(x0, T, u, 100);

% Plot results
figure
title('Cart response to sinusoidal force input');
subplot(3, 1, 1);
plot(t_traj, x_traj(1,:));
ylabel('Cart position (m)');
grid on
subplot(3, 1, 2);
plot(t_traj, x_traj(2,:));
ylabel('Cart velocity (m/s)');
grid on
subplot(3, 1, 3);
plot(t_traj, u_traj);
ylabel('Force input (N)');
grid on;
xlabel('Time (s)');

% Fit friction parameters
m = 0.40507;
ode = @(t, x, gamma) [x(2); (1/m)*(u(t)-1*mu(x(2), gamma))];
gamma = optFit(ode, [1.4022, 4.504, 1.8617, 2.2751, 4.0085, .02974], t_traj, x_traj, {'g1', 'g2', 'g3', 'g4', 'g5', 'g6'});

% Plot friction coefficient as function of velocity
v_max = 2*max(abs(x_traj(2,:)));
v = -v_max:0.01:v_max;
mu_v = mu(v, gamma);
figure;
plot(v, mu_v);
grid on;
xlabel('Velocity (m/s)');
ylabel('Friction coefficient');
title('Modelled friction coefficient');

% Model system friction using viscous damping only

ode = @(t, x, b) [x(2); (1/m)*(u(t)-b*x(2))];
b = optFit(ode, 8, t_traj, x_traj, {'b'});

return;

%%% Now test new model on real system %%%

% Forcing function
u = @(t, x) 7*sin(2*pi.*t);
T = 3;

% Simulate with viscous damping-only model
ode = @(t, x) [x(2); (1/m)*(u(t)-b*x(2))];
sol = ode45(ode, [0 T], x0);
t_traj_v_model = 0:0.01:T;
x_traj_v_model = deval(sol, t_traj_v_model);
u_traj_v_model = u(t_traj_v_model);

% Now ODE using new friction model with value of gamma found above
ode = @(t, x) [x(2); (1/m)*(u(t)-1*mu(x(2), gamma))];
sol = ode45(ode, [0 T], x0);
t_traj_f_model = 0:0.01:T;
x_traj_f_model = deval(sol, t_traj_f_model);
u_traj_f_model = u(t_traj_f_model);

% Open loop test on real system
disp('Testing open-loop response');
[t_traj_ol, x_traj_ol, u_traj_ol] = interface.followTrajectory(x_traj_f_model, T, u, 500);
% return;

% Closed loop test on real system
% Create system representation for TV-LQR
% Need a symbolic and numeric versions of system dynamics...


clear sys
sys.param.m = m;    % Do this _before_ creating the symbolic variable m
syms m b q1(t) u q1_dot mu_sym
% % q1_dot = diff(q1, 't');
% Symbolic representation of friction function
mu_sym = gamma(1)*(tanh(gamma(2)*q1_dot - tanh(gamma(3)*q1_dot))) + ...
    gamma(4)*tanh(gamma(5)*q1_dot) + gamma(6)*q1_dot;
% Symbolic representation of system dynamics
sys.x_dot_sym = [ q1_dot; (u - mu_sym*1)/m ];
% subs([ q1_dot; (u - mu_sym*1)/m ], diff(q1, 't'), sym('q1_dot'))

% clear q1 q1_dot
clear q1_dot;    % Get rid of time dependence
syms q1_dot q1;
% Get rid of time dependency, i.e. q1_dot rather than diff(q1(t), t))
sys.x_dot_sym = subs(sys.x_dot_sym, diff(q1, 't'), sym('q1_dot'));
% Need numeric version of ode, called like this: ode(t, x, u, param):

temp_fun = matlabFunction(sys.x_dot_sym, 'Vars',{t, [q1; q1_dot], u, m});
sys.x_dot_fun = @(t, x, u, param) temp_fun(t, x, u, param.m);
% sys.x_dot_fun = temp_fun;
sys.stateVars = sym([{'q1'}; {'q1_dot'}]);
sys.inputVars = u;
sys.nStates = 2;

% Create TV-LQR controller

lqr.Q = diag([10 1]);
lqr.R = .001;
lqr.Q_f = zeros(2, 2);
lqr.nSteps = length(x_traj_f_model); % Create a gain for every knot point
disp('Calculating finite horizon LQR gains');
[lqr, u_cl_fun, tIdxFun] = tvLqrDirCol(sys, lqr, [0 T], x_traj_f_model, u_traj_f_model);
% Follow trajectory with controller
[t_traj_cl, x_traj_cl, u_traj_cl] = interface.followTrajectory(x_traj_f_model, T, u_cl_fun, 500);

plotTrajComp({t_traj_v_model, t_traj_f_model, t_traj_ol, t_traj_cl}, {x_traj_v_model, x_traj_f_model, x_traj_ol, x_traj_cl}, 1, 2, ...
    [1 2 3 4], {':k', '--k', 'b', 'm'}, 'Cart system', ...
    {'q1', 'q1 dot'}, {'Viscous model', 'Friction model', 'Real system - OL', 'Real system - CL'});