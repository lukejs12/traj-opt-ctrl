% Test the EOM for the pendulum cart (distributed mass) system

clc;

disp('Loading system model');
load('SinglePendulumCartSys.mat', 'sys');

%% 'Correct' parameters - CAD/measurement/fitting
p.m1 = 0.24463;
p.c2 = 0.16951;
p.l2 = 0.3;
p.m2 = 0.12038;
p.I2 = 0.00246335160;
p.g = 9.81;
% Pre-lockdown:
% gamma_1 = [1.4022, 4.504, 1.8617, 2.2751, 4.0085, .02974];
% Re tested 18/10/20:
gamma_1 = [ 1.3947    4.4948    1.8766    2.2553    3.9928    0.0294];
gamma_2 = [0.050511, 0.0072542, 0.56505, 0.040219, 0.89881, 5.9566e-5];
p.g1_1 = gamma_1(1); p.g1_2 = gamma_1(2); p.g1_3 = gamma_1(3);
p.g1_4 = gamma_1(4); p.g1_5 = gamma_1(5); p.g1_6 = gamma_1(6);
p.g2_1 = gamma_2(1); p.g2_2 = gamma_2(2); p.g2_3 = gamma_2(3);
p.g2_4 = gamma_2(4); p.g2_5 = gamma_2(5); p.g2_6 = gamma_2(6);

% x0 = [0.0 0 0 0]';
% tspan = [0 2];
% % u = @(t) 0;
% % u = @(t) 3.*sin(1*2*pi*t);
% % u = @(t) -5 .* ((0<t) & (t<1.0));   % Safety testing - CAREFUL!
% % u = @(t) -6.* ((0<t) & (t<0.5));
% sol = ode45(@(t, x) sys.x_dot_fun(t, x, u(t), p), tspan, x0);
% tnom = 0:.05:tspan(end);
% T = tspan(end);
% xnom = deval(sol, tnom);
% unom = u(tnom);
% figure
% plot(tnom, xnom(1, :), 'k', tnom, xnom(2, :), 'b');
% grid on;
% AnimPendulumCart(xnom, tnom(end), p);
% return;

%% Load trajectory 
[xnom, unom, T, param, tmp] = loadTrajectory('Swingup160_181020.mat');
[~, nPoints] = size(xnom);
h = T/(nPoints-1);

% Nominal trajectory time vector
tnom = linspace(0, T, nPoints);
x0 = xnom(:, 1);

%% Add pause at end 
pauseAtEnd = false;
if pauseAtEnd == true
    % Add a pause at end of trajectory

    % Duration of pause in seconds
    Tp = 5;
    xnom = [xnom repmat(xnom(:, end), 1, Tp/h)];
    unom = [unom zeros(1, Tp/h)];
    tnom = [tnom tnom(end)+h:h:Tp+T];
    [~, nPoints] = size(xnom);
    T = T + Tp;
end

%% Stationary vertical trajectory
stayVerticalOnly = false;
if stayVerticalOnly == true
    T = 10;
    x0 = [0.2 pi 0 0]';
    xnom = repmat(x0, 1, T/h);
    nPoints = length(xnom);
    unom = zeros(1, nPoints);
    tnom = 0:h:T;
end


%% Now create controller for this trajectory
syms u;
sys.inputVars = u; % Needed for LQR. Need to resolve better.
sys.param = p;


% Create LQR structure
% lqr.Q = diag([1 16 1 16]);

% % THESE HAVE WORKED:
% lqr.Q = diag([.5 .05 1 1]);
% lqr.R = 1;
% lqr.Q_f = 0.1*eye(4);

% % THESE WORK PRETTY WELL
% lqr.Q = diag([.1 .1 0 .1]);
% lqr.R = 0.1;
% lqr.Q_f = diag([0 0 0 0]);

% Also work well
lqr.Q = diag([.05 .5 .05 .1]);
lqr.R = 0.08;
lqr.Q_f = diag([0 0 0 0]);

% % EXPERIMENTAL
% lqr.Q = diag([5 5 0 1]);
% lqr.R = 0.1;
% lqr.Q_f = diag([0 0 0 0]);

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

% Real system test

% % % Create a controller interface
% % disp('Creating pendulum controller interface');
% % interface = PendulumController;

% Closed loop controller
disp('');
input('Ready - hit [return] to proceed (system will move)');
% Follow the trajectory with closed loop TV-LQR
disp('CL controller');
[t_traj_cl, x_traj_cl, u_traj_cl] = interface.followTrajectory(xnom, T, u_cl_fun, 2000);

% Open loop controller
% disp('OL controller');
% [t_traj_ol, x_traj_ol, u_traj_ol] = interface.followTrajectory(xnom, T, u_ol, 500);

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

% Plot comparison of closed loop state trajectories
plotTrajComp({tnom, t_traj_cl}, {xnom, x_traj_cl}, 2, 2, ...
    [1 2 3 4], {':k', 'm'}, 'Double pendulum cart', ...
    {'q1', 'q2', 'q1 dot', 'q2 dot'}, {'Nominal', 'Real system (CL)'});
% Input force trajectories
plotTrajComp({tnom, t_traj_cl}, {unom, u_traj_cl}, 1, 1, ...
    [1], {':k', 'm'}, 'Cart system', ...
    {'u'}, {'Nominal', 'Closed loop'}); %#ok<NBRAK>
% AnimPendulumCart(x_traj_cl, tnom(end), p);

% % Plot comparison of open loop state trajectories
% plotTrajComp({tnom, t_traj_ol}, {xnom, x_traj_ol}, 2, 2, ...
%     [1 2 3 4], {':k', 'm'}, 'Double pendulum cart', ...
%     {'q1', 'q2', 'q1 dot', 'q2 dot'}, {'Nominal', 'Real system (OL)'});
% % Input force trajectories
% plotTrajComp({tnom, t_traj_ol}, {unom, u_traj_ol}, 1, 1, ...
%     [1], {':k', 'm'}, 'Cart system', ...
%     {'u'}, {'Nominal', 'Closed loop'}); %#ok<NBRAK>