
% Get current position
[theta, theta_dot, enabled, homed, estop, limit, t_limit] = interface.sendPacket(interface.CMD_NULL, 0);
x = interface.convertRawState([theta theta_dot]')

gains = [1 1 1 1];
x0 = [x(1:2); 0; 0];