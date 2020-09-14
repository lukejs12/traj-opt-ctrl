[m, n] = size(traj);
t = linspace(0, T, n);
f = figure;
hold on; axis equal;
xlim([-1.5 1.5]);
ylim([-.5 1]);

plot([-3 3], [0 0], 'k');
box = createBox(0, 0, .125, 0, .25, .125, 0, [.9 .5 .1]);
rod = createRod(traj(1, 1), .0625, 0, 0, .02, p.l2, traj(2, 1), 12, [.9 .9 0]);
% pause(1);

trail.num = 0;
trail.delay = .25;
trail.col = [.8 .8 .8];

animTraj(traj', t, p, {    {box, {'x', 'traj(n, 1)'}}, ...
                    {rod, {'x', 'traj(n, 1)'}, {'ang', 'traj(n, 2)+pi'}} ...
                }, trail, 1);