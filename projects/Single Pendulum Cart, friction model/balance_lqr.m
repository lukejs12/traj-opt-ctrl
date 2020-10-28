% Use an LQR to find gains for full state feedback, and balance
% the pendulum cart system upright, controlling cart position too
% System model uses 6-varible friction model for the cart and first 
% pendulum.


%% Load model, set all parameters
clc;
load('SinglePendulumCartSys.mat', 'sys');
% Grab system parameters from trajectory
[~, ~, ~, ~, ~, p_traj] = loadTrajectory('Swingup160_271020_2.mat');

% p.m1 = 0.24463;
% p.c2 = 0.16951;
% p.l2 = 0.3;
% p.m2 = 0.12038;
% p.I2 = 0.00246335160;
% p.g = 9.81;
% % 18/10/20 Measured after re-adjusting v bearings:
% gamma_1 = [ 1.3947    4.4948    1.8766    2.2553    3.9928    0.0294];
% % % Measured pre lockdown:
% % gamma_2 = [0.050511, 0.0072542, 0.56505, 0.040219, 0.89881, 5.9566e-5];
% % Measured 25/10/20:
% gamma_2 = [0.001285313      160.3206      82.04419  0.0004192054  9.364404e-07  0.0006945904];
% p.g1_1 = gamma_1(1); p.g1_2 = gamma_1(2); p.g1_3 = gamma_1(3);
% p.g1_4 = gamma_1(4); p.g1_5 = gamma_1(5); p.g1_6 = gamma_1(6);
% p.g2_1 = gamma_2(1); p.g2_2 = gamma_2(2); p.g2_3 = gamma_2(3);
% p.g2_4 = gamma_2(4); p.g2_5 = gamma_2(5); p.g2_6 = gamma_2(6);

sys.param = p_traj;
syms u;
sys.inputVars = u;

%% Create lists of system physical property names and values
paramNames = fieldnames(sys.param);
for n = 1:length(paramNames), paramVals(n) = getfield(sys.param, paramNames{n}); end

%% Create symbolic x_dot function substituting in physical parameters
physSys = subs(sys.x_dot_sym, paramNames, paramVals');

% Want to regulate about upright position. Pendulum angle (q2) is defined
% as 0 rad when pointing down, so redefine q2 := q2+pi so it's zero
% in upright position
physSys = subs(physSys, 'q2', str2sym('q2+pi'));

% Linearise system
A_lin_sym = jacobian(physSys, sys.stateVars); % df/dx
B_lin_sym = jacobian(physSys, sys.inputVars(1));  % df/du

% Create numeric versions of system model
A_lin_fun = matlabFunction(A_lin_sym, 'Vars', {[sys.stateVars; sys.inputVars]});
B_lin_fun = matlabFunction(B_lin_sym, 'Vars', {[sys.stateVars; sys.inputVars]});


x = [0 0 0 0]';
u = 0;
xu = [x; u];
A = A_lin_fun(xu);
B = B_lin_fun(xu);
C = [1 1 1 1];
D = 0;

%% Set up and compute LQR
% Q = diag([10 1 0 0]);
% Q = diag([10 1 0.5 0.5]);
% Q = diag([10 5 5 5]);
% R = 1;
% R = 0.125;
Q = diag([1 1 1 1]);
R = 10;

K = lqr(A, B, Q, R);

%% Create closed-loop system
ssSys = ss((A - B*K), B, C, D);

%% Simulate response
t = 0:0.01:20;
x0 = [0.1 -5*pi/180 0 0]';
[y, t, x] = initial(ssSys, x0, t);
% initial(sys, x0, t);

%% Plot
figure;

subplot(2, 1, 1);
plot(t, x(:, 1));
ylabel('Cart position');
subplot(2, 1, 2);
plot(t, x(:, 2));
ylabel('Pendulum angle');
xlabel('t (s)');
mtit('Linearized model with transposed coordinate system');

%% Simulate a different way to check understanding
ode = @(t, x) sys.x_dot_fun(t, x, -K*(x - [0 pi 0 0]'), sys.param);
[t, x] = ode45(ode, [0 t(end)], x0 + [0 pi 0 0]');
f = figure;
subplot(2, 1, 1);
plot(t, x(:, 1));
ylabel('Cart position');
subplot(2, 1, 2);
plot(t, x(:, 2));
ylabel('Pendulum angle');
xlabel('t (s)');
mtit(f, 'Non-linear model with original coordinate system');

disp(['K: ' num2str(K)])

% Create an observer

