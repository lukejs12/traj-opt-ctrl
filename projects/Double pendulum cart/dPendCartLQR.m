% dPendCartLQR

disp('Loading system model');
load('doublePendCartSys.mat', 'sys');
syms u;
sys.inputVars = u; % Needed for LQR. Need to resolve better.
% 3/4 in x 18 swg
p.m1 = 0.125; p.b1 = 0;%0.01;
p.m2 = 0.098; p.I2 = 0.00205; p.c2 = 0.20956; p.l2 = 0.422; p.b2 = 0.00125;
% 2nd pendulum with bob at end
p.m3 = 0.13971; p.I3 = 0.0030865; p.c3 = 0.26557; p.l3 = 0.4; p.b3 = 0.005;
% % 2nd pendulum w/o bob
% p.m3 = 0.08646; p.I3 = 0.001446; p.c3 = 0.18; p.l3 = 0.411; p.b3 = 0.00125;
p.g = 9.81;
sys.param = p;

disp('Loading nominal trajectory');
[xnom, unom, T, param, tmp] = loadTrajectory('doublePendCart_240_dircol_1Tsq_1usq_50uMx.mat');   %doublePendCart_150_dircol_1usq_25uMx  doublePendCart_240_dircol_1Tsq_1usq_50uMx
%'doublePendCart_120_dircol_10Tsq_0_25usq_40uMx'); % Works
[~, nPoints] = size(xnom);
h = T/(nPoints-1);
% Nominal trajectory time vector
t0 = linspace(0, T, nPoints);

% Create LQR structure
% lqr.Q = 100*eye(6);
lqr.Q = .1*diag([1 5 5 0 0 0]);
% lqr.Q = diag([1 2.5 2.5 .5 .5 .5]);
lqr.R = 1;
% lqr.Q_f = .5*eye(6);%diag([5 5 5 1 1 1]); %5*eye(sys.nStates);
lqr.Q_f = lqr.Q; %diag([1 5 5 .5 .5 .5]);

lqr.nSteps = nPoints; % Create a gain for every knot point
% Get time varying LQR controller
disp('Calculating finite horizon LQR gains');
% [lqr, u_cl_fun, x0_p, u0_p, tIdxFun] = tvLqr(sys, lqr, [0 T], xnom, unom);
[lqr, u_cl_fun, tIdxFun] = tvLqrDirCol(sys, lqr, [0 T], xnom, unom);
% return;
% Change initial state
x_zero = [0 0 0 0 0 0];
% x_zero = [-.5 -30*pi/180 30*pi/180 0 0 0]';
% % Perturb system physical properties
% sys.param.b1 = 0.25;
% sys.param.b2 = 0.0075;
% sys.param.m2 = sys.param.m2*1.05;
% sys.param.b3 = sys.param.b3*2;
% sys.param.m3 = sys.param.m3*1.05;

% sys.param.b
% ZOH input function
u_ol = @(t, x) unom(round(t/h) + 1);
% u_ol = @(t, x) xuOft(
disp('Simulating open- and closed-loop trajectories of perturbed system');

% Open loop simulation with as many steps as nominal trajectory
% [t_vect_ol, x_traj_ol, u_traj_ol] = rk4(@(t, x, u) sys.x_dot_fun(t, x, u, sys.param), u_ol, [t0(1) t0(end)], x_zero, nKnotPoints-1);
[t_vect_ol, x_traj_ol] = ode45(@(t, x) sys.x_dot_fun(t, x, u_ol(t, x), sys.param), [t0(1) t0(end)], x_zero);

% Closed loop simulation
[t_vect_cl, x_traj_cl, u_traj_cl] = rk4(@(t, x, u) sys.x_dot_fun(t, x, u, sys.param), u_cl_fun, [0 T], x_zero, lqr.nSteps*10);
[t_vect_cl2, x_traj_cl2] = ode45(@(t, x) sys.x_dot_fun(t, x, u_cl_fun(t, x), sys.param), [t0(1) t0(end)], x_zero);

disp('Plot system response comparison');
% Plot comparison of state trajectories
plotTrajComp({t0, t_vect_ol, t_vect_cl}, {xnom, x_traj_ol, x_traj_cl}, 2, 3, ...
    [1 2 3 4 5 6], {':k', 'b', 'm'}, 'Double pendulum cart', ...
    {'q1', 'q2', 'q3', 'q1 dot', 'q2 dot', 'q3 dot'}, {'Nominal', 'Open loop', 'Closed loop'});
% Input force trajectories
% plotTrajComp({t0, t_vect_ol, t_vect_cl}, {unom, u_traj_ol, u_traj_cl}, 1, 1, ...
%     [1], {':k', 'b', 'm'}, 'Pendulum cart (point mass)', ...
%     {'u'}, {'Nominal', 'Open loop', 'Closed loop'}); %#ok<NBRAK>
plotTrajComp({t0, t_vect_cl}, {unom, u_traj_cl}, 1, 1, ...
    [1], {'b', 'm'}, 'Pendulum cart (point mass)', ...
    {'u'}, {'Open loop', 'Closed loop'}); %#ok<NBRAK>