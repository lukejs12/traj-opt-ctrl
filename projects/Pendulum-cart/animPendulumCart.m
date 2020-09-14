[m, n] = size(traj);
t = linspace(0, T, n);
f = figure;
hold on; axis equal;
xlim([-3 3]);
ylim([-.5 1.5]);
param.l = .5;

plot([-3 3], [0 0], 'k');
box = createBox(0, 0, .25, 0, .5, .25, 0, [.9 .5 .1]);
rod = createRod(traj(1, 1), .125, 0, 0, .025, param.l, traj(2, 1), 12, [.9 .9 0]);

trail.num = 0;
trail.delay = .125;
trail.col = [.8 .8 .8];

animTraj(traj', t, param, {    {box, {'x', 'traj(n, 1)'}}, ...
                    {rod, {'x', 'traj(n, 1)'}, {'ang', 'traj(n, 2)+pi'}} ...
                }, trail, 1);