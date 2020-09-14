% Create a controller interface
clear variables;
disp('Creating pendulum controller interface');
interface = PendulumController;

% Home the cart-pendulum system
disp('Homing');
interface.Home();

% Enable the motor
[theta, theta_dot, enabled, homed, estop, limit, t_limit] = interface.sendPacket(interface.CMD_ENABLE, 0);
i = 1;
FREQUENCY = 200;
PERIOD = 1/FREQUENCY;
DURATION = 5;
x_traj = zeros(4, DURATION/PERIOD);
t_traj = zeros(1, DURATION/PERIOD);
t_start = tic;
t_nom = 0;
t_act = 0;
while t_act < DURATION
    % Wait for the 
    while toc(t_start) < t_nom; end
    t_nom = t_nom + PERIOD;
    % Get state
    [theta, theta_dot, enabled, homed, estop, limit, t_limit] = interface.sendPacket(interface.CMD_NULL, 0);
    x = interface.ConvertRawState([theta theta_dot]');
    x = [x(1:2); x(4:5)];
    t_act = toc(t_start);
    x_traj(:, i) = x;
    t_traj(i) = t_act;
    i = i + 1;
    if rem(t_act, 0.5) < PERIOD/2
        disp([num2str(t_act) ', [' num2str(theta_dot) ']']);
    end
end
interface.sendPacket(interface.CMD_DISABLE, 0);
interface.delete();

figure;
subplot(4, 1, 1);
plot(t_traj, x_traj(1,:));

subplot(4, 1, 2);
plot(t_traj, x_traj(2,:));

subplot(4, 1, 3);
plot(t_traj, x_traj(3,:));

subplot(4, 1, 4);
plot(t_traj, x_traj(4,:));