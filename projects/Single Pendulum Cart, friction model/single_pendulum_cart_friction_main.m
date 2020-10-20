clear all;
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
% gamma_1 = [1.4022, 4.504, 1.8617, 2.2751, 4.0085, .02974];
gamma_2 = [0.050511, 0.0072542, 0.56505, 0.040219, 0.89881, 5.9566e-5];
p.g1_1 = gamma_1(1); p.g1_2 = gamma_1(2); p.g1_3 = gamma_1(3);
p.g1_4 = gamma_1(4); p.g1_5 = gamma_1(5); p.g1_6 = gamma_1(6);
p.g2_1 = gamma_2(1); p.g2_2 = gamma_2(2); p.g2_3 = gamma_2(3);
p.g2_4 = gamma_2(4); p.g2_5 = gamma_2(5); p.g2_6 = gamma_2(6);

sys.param = p;

% % % If parameters change, run these two lines to update gradient function
% % tic
% % sys = createDircolNlConGrads2(sys);
% % toc
% % save([sys.name 'Sys.mat'], 'sys');
% % return

method = 'dircol';
gradients = 'centraldiff'; % solvergrads/centraldiff/analytic
nPoints = 160;
x0 = [-0.3 0 0 0]';
xf = [0.3 pi 0 0]';
[guess.traj, guess.u, guess.T, ~, ~] = loadTrajectory('Swingup40', nPoints);
% guess = 0;
xLims = [-0.4 -2*pi -Inf -Inf; 0.4 2*pi Inf Inf]';

uMax = 30;
tLims = [1.5 1.5];
cost.u = 1;
cost.T = 0;
cost.accSmooth = 0;
cost.uSmooth = 0;

[traj, u, T, param, exitflag, output] = trajOpt(sys, method, gradients, cost, nPoints, x0, xf, guess, xLims, uMax, tLims);
% autosaveTrajectory
t = linspace(0, T, nPoints);
figure;
subplot(2, 1, 1);
plot(t, traj(1,:), t, traj(3, :))
ylabel('q1, q2');
legend('q1 (m)', 'q2 (rad)');
grid on;
subplot(2, 1, 2)
plot(t, u);
ylabel('u (N)');
grid on;
xlabel('t (s)');

AnimPendulumCart(traj, T, p);