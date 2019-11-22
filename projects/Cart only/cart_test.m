% Test the EOM for the pendulum cart (distributed mass) system
% clear variables
clc;
x0 = [0 0]';
tspan = [0 15];
p.m = 0.24463;
p.b = 1.5;

% Forcing function
u = @(t) 4.*sin(1*2*pi*t);
% [t, y] = ode45(@(t, x) SinglePendulumCartEom(t, x, u(t), p), tspan, x0);
sol = ode45(@(t, x) CartEom(t, x, u(t), p), tspan, x0);
t = 0:.05:tspan(end);
T = tspan(end);
traj = deval(sol, t);
figure
plot(t, traj(2, :));%, 'k', t, y(:, 2), 'b')
% AnimCart
% return
% Now create controller for this trajectory

disp('Loading system model');
% [xnom, unom, T, param, tmp] = loadTrajectory('1013.mat');   %, doublePendCart_240_dircol_1Tsq_1usq_50uMx, doublePendCart_150_dircol_1usq_25uMx  doublePendCart_240_dircol_1Tsq_1usq_50uMx
load('CartSys.mat', 'sys');
xnom = traj;
unom = u(t);
syms u;
sys.inputVars = u; % Needed for LQR. Need to resolve better.
sys.param = p;
[~, nPoints] = size(xnom);
h = T/(nPoints-1);
% Nominal trajectory time vector
t0 = linspace(0, T, nPoints);

% Create LQR structure
lqr.Q = diag([10 1]);
% lqr.Q = diag([1 1]);
lqr.R = .001;
lqr.Q_f = zeros(2, 2);%lqr.Q;
lqr.nSteps = nPoints; % Create a gain for every knot point
disp('Calculating finite horizon LQR gains');
% [lqr, u_cl_fun, x0_p, u0_p, tIdxFun] = tvLqr(sys, lqr, [0 T], xnom, unom);
[lqr, u_cl_fun, tIdxFun] = tvLqrDirCol(sys, lqr, [0 T], xnom, unom);
% return;
% ZOH input function
u_ol = @(t, x) unom(round(t/h) + 1);
% u_ol = @(t, x) xuOft(
disp('Simulating open- and closed-loop trajectories of perturbed system');

% % Open loop simulation with as many steps as nominal trajectory
% ol_sol = ode45(@(t, x) sys.x_dot_fun(t, x, u_ol(t, x), sys.param), [t0(1) t0(end)], x_zero);
% x_traj_ol = deval(ol_sol, t0);
x_traj_ol = traj;
t_traj_ol = t;
 
% Closed loop simulation
cl_sol_sim = ode45(@(t, x) sys.x_dot_fun(t, x, u_cl_fun(t, x), sys.param), [t0(1) t0(end)], x0);
x_traj_cl_sim = deval(cl_sol_sim, t);
% Closed loop test

% Create a controller interface
% % % disp('Creating pendulum controller interface');
% % % interface = PendulumController;

disp('');
input('Ready - hit [return] to proceed (system will move)');
% Follow the trajectory
[t_traj_cl, x_traj_cl, u_traj_cl] = interface.FollowTrajectory(xnom, t(end), u_cl_fun, 500);

% % % interface.delete();

disp('Plot system response comparison');
% Plot comparison of state trajectories
plotTrajComp({t0, t0, t, t_traj_cl}, {xnom, x_traj_ol, x_traj_cl_sim, x_traj_cl}, 1, 2, ...
    [1 2 3 4 5 6], {':k', 'b', 'm', 'c'}, 'Cart system', ...
    {'q1', 'q1 dot'}, {'Nominal', 'Open loop', 'Closed loop (sim)', 'Closed loop (real)'});
str = ['LQR - Q: ' num2str(reshape(lqr.Q, 1, numel(lqr.Q))) ', R: ' num2str(lqr.R) ', Q_f: ' num2str(reshape(lqr.Q_f, 1, numel(lqr.Q_f)))];
text(0, 0, str)
% Input force trajectories
plotTrajComp({t0, t_traj_cl}, {unom, u_traj_cl}, 1, 1, ...
    [1], {':k', 'm'}, 'Cart system', ...
    {'u'}, {'Nominal', 'Closed loop'}); %#ok<NBRAK>