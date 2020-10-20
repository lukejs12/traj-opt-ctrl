% Use an LQR to find gains for full state feedback, and balance
% the pendulum cart system upright, controlling cart position too
% System model uses 6-varible friction model for the cart and first 
% pendulum.


%% Load model, set all parameters
clc;
load('SinglePendulumCartSys.mat', 'sys');

p.m1 = 0.24463;
p.c2 = 0.16951;
p.l2 = 0.3;
p.m2 = 0.12038;
p.I2 = 0.00246335160;
p.g = 9.81;
% 18/10/20 Measured after re-adjusting v bearings:
gamma_1 = [ 1.3947    4.4948    1.8766    2.2553    3.9928    0.0294];
% Measured pre lockdown:
gamma_2 = [0.050511, 0.0072542, 0.56505, 0.040219, 0.89881, 5.9566e-5];
p.g1_1 = gamma_1(1); p.g1_2 = gamma_1(2); p.g1_3 = gamma_1(3);
p.g1_4 = gamma_1(4); p.g1_5 = gamma_1(5); p.g1_6 = gamma_1(6);
p.g2_1 = gamma_2(1); p.g2_2 = gamma_2(2); p.g2_3 = gamma_2(3);
p.g2_4 = gamma_2(4); p.g2_5 = gamma_2(5); p.g2_6 = gamma_2(6);
sys.param = p;
syms u;
sys.inputVars = u;

%% Create lists of system physical property names and values
paramNames = fieldnames(sys.param);
for n = 1:length(paramNames), paramVals(n) = getfield(sys.param, paramNames{n}); end

%% Create symbolic x_dot function substituting in physical parameters
physSys = subs(sys.x_dot_sym, paramNames, paramVals');

% Linearise system
A_lin_sym = jacobian(physSys, sys.stateVars); % df/dx
B_lin_sym = jacobian(physSys, sys.inputVars(1));  % df/du

% Create numeric versions of system model
A_lin_fun = matlabFunction(A_lin_sym, 'Vars', {[sys.stateVars; sys.inputVars]});
B_lin_fun = matlabFunction(B_lin_sym, 'Vars', {[sys.stateVars; sys.inputVars]});

x = [0 pi 0 0]';
u = 0;
xu = [x; u];
A_lin = A_lin_fun([0, pi, 0, 0, 0]')
B_lin = B_lin
