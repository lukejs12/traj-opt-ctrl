% Differentiate position to get velocity, compare to telemetry
q2_dot = zeros(1, length(x_traj)-1);
for i = 1:length(x_traj)-1
    q2_dot(i) = (x_traj(2, i+1) - x_traj(2, i))/(t_traj(i+1) - t_traj(i));
end
figure
plot(t_traj, x_traj(5, :)*180/pi, t_traj(1:end-1), q2_dot*180/pi);
grid on
legend('Telemetry rate (deg/s)', 'Differentiated position (deg)');
xlabel('Time (s)');
ylabel('Angle (deg/s)')
% plot(t_traj_cl(1:end-1), q2_dot);
% plot(t_traj_cl, x_traj_cl(4, :));

% Integrate velocity to get position, compare
int_position = zeros(1, length(x_traj));
int_position(1) = x_traj(2, 1);
for i = 2:length(x_traj)
    h = t_traj(i) - t_traj(i-1);
    int_position(i) = int_position(i-1)+x_traj(5, i-1)*h;
end
figure
hold on; grid on;
plot(t_traj, x_traj(2, :)*180/pi);
plot(t_traj, int_position*180/pi);
legend('Teletry angle', 'Integrated angle');
xlabel('Time (s)');
ylabel('Angle (deg)')