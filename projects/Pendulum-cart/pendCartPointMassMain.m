clear all;
load('pendulumCartPointMassSys.mat', 'sys');

% p.g = 9.81;
% p.l = .5;
% p.m1 = .5;
% p.m2 = .5;
% p.b = 0;
% sys.param = p;

% If parameters change, run these two lines to update gradient function
% tic
% sys = createDircolNlConGrads2(sys);
% toc
% save([sys.name 'Sys.mat'], 'sys');
% return

p.m1 = 2.0; p.m2 = 0.5; p.g = 9.81; p.l = 0.5; % p.b = 0;
sys.param = p;

method = 'dircol';
gradients = 'analytic'; % solvergrads/centraldiff/analytic
nPoints = 80;
x0 = [0 0 0 0]';
% xf = [0.8 pi 0 0]';
xf = [0 pi 0 0]';

% guess.traj = (xf-x0)*linspace(0, 1, nPoints);
% guess.T = 3;
% guess.traj(4,:) = (pi/guess.T)*ones(1, nPoints);
% guess.u = zeros(1, nPoints);
% guess = 0;
% guess = 'pendulumCartPointMass_80_dircol_1usq_100uMx_(2)';

guess = 'pendulumCartPointMass_30_dircol_1usq_100uMx';

xLims = [-2*.8 -2*pi -Inf -Inf; 2*.8 2*pi Inf Inf]';
% xLims = [-2*.8 -Inf -Inf -Inrf; 2*.8 Inf Inf Inf]';
uMax = 100;
tLims = [2 2];
% tLims = [.1 5];
cost.u = 1;%1/nPoints;
cost.T = 0;
cost.accSmooth = 0;
cost.uSmooth = 0;
[traj, u, T, param, exitflag, output] = trajOpt(sys, method, gradients, cost, nPoints, x0, xf, guess, xLims, uMax, tLims);