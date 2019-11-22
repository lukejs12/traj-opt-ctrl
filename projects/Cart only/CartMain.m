clear all;
load('SinglePendulumCartSys.mat', 'sys');

p.m1 = 0.24463;
p.b1 = 3;

p.c2 = 0.13248;
p.l2 = 0.3;
p.m2 = 0.09465;
p.b2 = 0.0035;
p.I2 = 0.00353067843;
p.g = 9.81;
sys.param = p;

% % % If parameters change, run these two lines to update gradient function
% % tic
% % sys = createDircolNlConGrads2(sys);
% % toc
% % save([sys.name 'Sys.mat'], 'sys');
% % return

method = 'dircol';
gradients = 'centraldiff'; % solvergrads/centraldiff/analytic
nPoints = 80;
x0 = [0 0 0 0]';
% xf = [0.8 pi 0 0]';
xf = [0 pi 0 0]';

% guess.traj = (xf-x0)*linspace(0, 1, nPoints);
% guess.T = 3;
% guess.traj(4,:) = (pi/guess.T)*ones(1, nPoints);
% guess.u = zeros(1, nPoints);
[guess.traj, guess.u, guess.T, ~, ~] = loadTrajectory('SinglePendulumCart_30_dircol_1usq_20uMx_(3)', nPoints);
% guess = 0;
% guess = 'pendulumCartPointMass_80_dircol_1usq_100uMx_(2)';


xLims = [-0.5 -pi -Inf -Inf; 0.5 2*pi Inf Inf]';
% xLims = [-2*.8 -Inf -Inf -Inrf; 2*.8 Inf Inf Inf]';
uMax = 10;
tLims = [5 5];
% tLims = [.1 5];
cost.u = 1;%1/nPoints;
cost.T = 0;
cost.accSmooth = 0;
cost.uSmooth = 1;
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