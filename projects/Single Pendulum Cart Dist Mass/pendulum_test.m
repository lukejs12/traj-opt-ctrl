% Test the EOM for the pendulum cart (distributed mass) system

% clear variables
clc;

% % 'Correct' parameters - CAD/measurement/fitting
p.m1 = 0.24463;
p.b1 = 7.941;       % fmincon - 11/12/19
% p.b1 = 3.6058;       % Exponential curve fitting estimate
p.c2 = 0.16951;
p.l2 = 0.3;
p.m2 = 0.12038;
p.b2 = 5.2788e-16;  % fmincon - 11/12/19
% p.b2 = 0.0012354;      % Exponential curve fitting estimate
p.I2 = 0.00246335160;
p.g = 9.81;

% Create test forcing function and trajectory from ODE simulation
% x0 = [0 0 0 0]';
% tspan = [0 15];
% u = @(t) 3.*sin(1*2*pi*t);
% sol = ode45(@(t, x) SinglePendulumCartEom(t, x, u(t), p), tspan, x0);
% tnom = 0:.05:tspan(end);
% T = tspan(end);
% xnom = deval(sol, tnom);
% unom = u(tnom);
% figure
% plot(t, traj(1, :), 'k', t, traj(2, :), 'b');
% grid on;
% AnimPendulumCart

disp('Loading system model');
load('SinglePendulumCartSys.mat', 'sys');

% % Load trajectory 
[xnom, unom, T, param, tmp] = loadTrajectory('SwingUp3s.mat');
x0 = xnom(:, 1);

% Now create controller for this trajectory
syms u;
sys.inputVars = u; % Needed for LQR. Need to resolve better.
sys.param = p;

[~, nPoints] = size(xnom);
h = T/(nPoints-1);
% Nominal trajectory time vector

% Create LQR structure
% lqr.Q = .1*eye(6);
% lqr.Q = .0001*diag([1 5 5 1 5 5]);
% lqr.Q = diag([2 2.5 .05 .1]);
% lqr.Q = diag([1 2 1 2]);
% lqr.R = 0.25;
% lqr.Q_f = zeros(4, 4);%lqr.Q;

lqr.Q = diag([1 16 1 16]);
lqr.R = 1;
lqr.Q_f = eye(4);
% lqr.Q_f = lqr.Q;
% lqr.Q_f = zeros(4, 4);


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
% ol_sol = ode45(@(t, x) sys.x_dot_fun(t, x, u_ol(t, x), sys.param), [tnom(1) tnom(end)], x0);

% ol_sol = ode45(@(t, x) SinglePendulumCartEom(t, x, u_ol(t, x), p), [t0(1) t0(end)], x0);

% x_traj_ol = deval(ol_sol, tnom);
 
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
% Follow the trajectory with closed loop TV-LQR
disp('CL controller');
[t_traj_cl, x_traj_cl, u_traj_cl] = interface.followTrajectory(xnom, T, u_cl_fun, tIdxFun, 500);

% % Follow the trajectory with open loop controller
% disp('OL controller');
% [t_traj_cl, x_traj_cl, u_traj_cl] = interface.followTrajectory(xnom, T, u_ol, tIdxFun, 500);

% % interface.delete();

% disp('Plot system response comparison');
% % Plot comparison of state trajectories
% plotTrajComp({tnom, tnom, t_traj_cl}, {xnom, x_traj_ol, x_traj_cl}, 2, 2, ...
%     [1 2 3 4], {':k', 'b', 'm'}, 'Double pendulum cart', ...
%     {'q1', 'q2', 'q1 dot', 'q2 dot'}, {'Nominal', 'Open loop (ODE)', 'Closed loop (real system)'});
% % Input force trajectories
% plotTrajComp({tnom, t_traj_cl}, {unom, u_traj_cl}, 1, 1, ...
%     [1], {':k', 'm'}, 'Cart system', ...
%     {'u'}, {'Nominal', 'Closed loop'}); %#ok<NBRAK>

tnom = linspace(0, T, nPoints);
% Plot comparison of state trajectories
plotTrajComp({tnom, t_traj_cl}, {xnom, x_traj_cl}, 2, 2, ...
    [1 2 3 4], {':k', 'm'}, 'Double pendulum cart', ...
    {'q1', 'q2', 'q1 dot', 'q2 dot'}, {'Nominal', 'Real system)'});
% Input force trajectories
plotTrajComp({tnom, t_traj_cl}, {unom, u_traj_cl}, 1, 1, ...
    [1], {':k', 'm'}, 'Cart system', ...
    {'u'}, {'Nominal', 'Closed loop'}); %#ok<NBRAK>