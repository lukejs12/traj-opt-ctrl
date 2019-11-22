% Test the EOM for the pendulum cart (distributed mass) system
% clear variables
clc;
x0 = [-0.25 0 0 0]';
tspan = [0 15];
p.m1 = 0.24463;
p.b1 = 2;

p.c2 = 0.13248;
p.l2 = 0.3;
p.m2 = 0.09465;
p.b2 = 0.0035;
p.I2 = 0.00353067843;
p.g = 9.81;

% Forcing function
u = @(t) 3.*sin(1*2*pi*t);
sol = ode45(@(t, x) SinglePendulumCartEom(t, x, u(t), p), tspan, x0);
t = 0:.05:tspan(end);
T = tspan(end);
traj = deval(sol, t);
% figure
% plot(t, traj(1, :), 'k', t, traj(2, :), 'b');
% grid on;
% AnimPendulumCart

disp('Loading system model');
load('SinglePendulumCartSys.mat', 'sys');

% Load trajectory
% xnom = traj;
% unom = u(t);
[xnom, unom, T, param, tmp] = loadTrajectory('SinglePendulumCart_80_dircol_1usq_1uSmooth_20uMx.mat');


% Now create controller for this trajectory
syms u;
sys.inputVars = u; % Needed for LQR. Need to resolve better.
sys.param = p;

%'doublePendCart_120_dircol_10Tsq_0_25usq_40uMx'); % Works
[~, nPoints] = size(xnom);
h = T/(nPoints-1);
% Nominal trajectory time vector
t0 = linspace(0, T, nPoints);

% Create LQR structure
% lqr.Q = .1*eye(6);
% lqr.Q = .0001*diag([1 5 5 1 5 5]);
% lqr.Q = diag([2 2.5 .05 .1]);
lqr.Q = diag([1 1 1 1]);
lqr.R = 1;
lqr.Q_f = zeros(4, 4);%lqr.Q;

lqr.nSteps = nPoints; % Create a gain for every knot point
% Get time varying LQR controller
disp('Calculating finite horizon LQR gains');
% [lqr, u_cl_fun, x0_p, u0_p, tIdxFun] = tvLqr(sys, lqr, [0 T], xnom, unom);
[lqr, u_cl_fun, tIdxFun] = tvLqrDirCol(sys, lqr, [0 T], xnom, unom);
% return;

% ZOH input function
u_ol = @(t, x) unom(round(t/h) + 1);
% u_ol = @(t, x) xuOft(
disp('Simulating open- and closed-loop trajectories of perturbed system');

% Open loop simulation with as many steps as nominal trajectory
% [t_vect_ol, x_traj_ol, u_traj_ol] = rk4(@(t, x, u) sys.x_dot_fun(t, x, u, sys.param), u_ol, [t0(1) t0(end)], x_zero, nKnotPoints-1);
ol_sol = ode45(@(t, x) sys.x_dot_fun(t, x, u_ol(t, x), sys.param), [t0(1) t0(end)], x0);
x_traj_ol = deval(ol_sol, t0);
 
% % Closed loop simulation
% % [t_vect_cl, x_traj_cl, u_traj_cl] = rk4(@(t, x, u) sys.x_dot_fun(t, x, u, sys.param), u_cl_fun, [0 T], x_zero, lqr.nSteps*10);
% cl_sol = ode45(@(t, x) sys.x_dot_fun(t, x, u_cl_fun(t, x), sys.param), [t0(1) t0(end)], x_zero);
% x_traj_cl = deval(cl_sol, t0);

% Closed loop test

% % % Create a controller interface
% % disp('Creating pendulum controller interface');
% % interface = PendulumController;

disp('');
input('Ready - hit [return] to proceed (system will move)');
% Follow the trajectory
[t_traj_cl, x_traj_cl, u_traj_cl] = interface.FollowTrajectory(xnom, t(end), u_cl_fun, tIdxFun, 500);

% % interface.delete();

disp('Plot system response comparison');
% Plot comparison of state trajectories
plotTrajComp({t0, t0, t_traj_cl}, {xnom, x_traj_ol, x_traj_cl}, 2, 2, ...
    [1 2 3 4], {':k', 'b', 'm'}, 'Double pendulum cart', ...
    {'q1', 'q2', 'q1 dot', 'q2 dot'}, {'Nominal', 'Open loop', 'Closed loop'});
% Input force trajectories
plotTrajComp({t0, t_traj_cl}, {unom, u_traj_cl}, 1, 1, ...
    [1], {':k', 'm'}, 'Cart system', ...
    {'u'}, {'Nominal', 'Closed loop'}); %#ok<NBRAK>