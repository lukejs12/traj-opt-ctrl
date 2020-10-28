clear variables;
clc;
load('SinglePendulumCartSys.mat', 'sys');

p.m1 = 0.24463;
p.c2 = 0.16951;
p.l2 = 0.3;
p.m2 = 0.12038;
p.I2 = 0.00246335160;   % About COM
p.g = 9.81;
p.b2 = 0.00042253;
p.b1 = 0; %???


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