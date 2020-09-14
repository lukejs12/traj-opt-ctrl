% Derive equations in System 2 (notebook 19/1/20)
% Manually deriving friction / damping terms

clear variables
syms m1 m2 Fx k t x1(t) x1_dot(t) x2(t) x2_dot(t) b mu_1_2 g u
% gamma2 = sym('g', [1, 6]);
x1_dot = diff(x1, 't') %#ok<*NOPTS>
x2_dot = diff(x2, 't')
T = 0.5*m1*x1_dot^2 + 0.5*m2*x2_dot^2
V = 0.5*k*x2^2
L = T-V;
U = [u; 0]
mu = @(qd, gamma) gamma(1)*(tanh(gamma(2)*qd - tanh(gamma(3)*qd))) + ...
    gamma(4)*tanh(gamma(5)*qd) + gamma(6)*qd;
gamma1 = sym('g1_', [1, 6]);
Q = [-1*b*x1_dot - m2*g*mu(x1_dot-x2_dot, gamma1); m2*g*mu(x1_dot-x2_dot, gamma1)]
D = 0;
% mu_1_2_v(v) = gma(1)*(tanh(gma(2)*v) - tanh(gma(3)*v)) + gma(4)*tanh(gma(5)*v) + gma(6)*v
coordVars = {x1, x2};
sys = deriveEom('System_2', coordVars, L, D, U, Q, true)
% sys = deriveEom('System_2', coordVars, T, D, N_forces, mu_funcs, v_funcs, U, false)
x0 = [0; -0.5; 0; 0];
p.b = 10000; p.mu_1_2 = 20; p.g = 9.81; p.k = 5; p.m1 = 0.5; p.m2 = 0.5;
% gamma = [0.0505 0.007254 0.56505 0.04022 0.89881 5.9566e-5];
% gamma = [0 0 0 1 10 0]; % Coulombic
gamma = [0 0 0 0 0 0.1]; % Viscous
p.g1_1 = gamma(1); p.g1_2 = gamma(2); p.g1_3 = gamma(3); 
p.g1_4 = gamma(4); p.g1_5 = gamma(5); p.g1_6 = gamma(6); 
sol = ode45(@(t, x) sys.x_dot_fun(t, x, 0, p), [0 10], x0);
t = 0:.1:10;
x_traj = deval(sol, t);
figure;
plot(t, x_traj(1,:), t, x_traj(2,:));
legend('m1 position', 'm2 position');
grid on;

% Plot friction coefficient
v_max = 3;
v = -v_max:0.01:v_max;
mu_v = mu(v, gamma);
figure;
plot(v, mu_v);
grid on;
xlabel('Velocity (m/s)');
ylabel('Friction coefficient');
title('Modelled friction coefficient');


% mup =  sym('gamma', [7 1])
% mu_sym = symfun(mup(1)*(tanh(mup(2)*mup(7) - tanh(mup(3)*mup(7)))) + ...
%     mup(4)*tanh(mup(5)*mup(7)) + mup(6)*mup(7),mup);
% mu = @(qd, gma) mup(gma(1), gma(2), gma(3), gma(4), gma(5), gma(6), qd)
% mu(5, [1 2 3 4 5 6])
% 
% clear variables
% 
% mu = @(qd, gamma) gamma(1)*(tanh(gamma(2)*qd - tanh(gamma(3)*qd))) + ...
%     gamma(4)*tanh(gamma(5)*qd) + gamma(6)*qd;
% 
% 
% Deriving equations in example 2.1 (The Conveyor Belt) from paper 
% Rayleigh’s dissipation function at work

% clear all
% syms mu_0 mu_inf a v_r mu(v_r) t x(t) y(t) x_dot v_0 y_dot  N v m
% % v_r = sqrt((x_dot - v_0)^2 + y_dot^2)
% x_dot = diff(x, 't')
% y_dot = diff(y, 't')
% v_r = sqrt((x_dot - v_0)^2 + y_dot^2)
% mu(v) = (mu_0 - mu_inf)/(1 + a*v) + mu_inf
% Q = -N*mu(v_r)*subs(diff(subs(v_r, diff(x, t), 'tempVar'), 'tempVar'), 'tempVar', diff(x, t));
% Q = simplify(Q, 200)
% pretty(Q)

% Test evalLagrangianDissipation

% clear variables;
% syms mu_0 mu_inf a v_r mu(v_r) t x(t) y(t) x_dot v_0 y_dot  N v m
% x_dot = diff(x, 't');
% y_dot = diff(y, 't');
% T = (m/2)*(x_dot^2 + y_dot^2)
% V = 0;
% v_r = sqrt((x_dot - v_0)^2 + y_dot^2);
% v_funcs = {v_r, v_r}
% N_forces = {N, N}
% mu(v) = (mu_0 - mu_inf)/(1 + a*v) + mu_inf
% mu_funcs = {mu, mu};
% coordVars = {x, y};
% syms Fx Fy
% U = {Fx; Fy};
% % eom = evalLagrangianDissipation(T, V, N_forces, mu_funcs, v_funcs, coordVars)
% sys = deriveEom('Conveyor', coordVars, T, V, N_forces, mu_funcs, v_funcs, U, false)

% % System 1 from notebook
% clear variables
% syms m Fx k b t x(t) x_dot(t) v mu_v(v)
% x_dot = diff(x, 't')
% T = 0.5*m*x_dot^2
% V = 0.5*k*x^2
% v_funcs = {x_dot}
% N_forces = {1}
% mu_v(v) = b*v
% mu_funcs = {mu_v}
% coordVars = {x};
% U = {Fx}
% sys = deriveEom('System_1', coordVars, T, V, N_forces, mu_funcs, v_funcs, U, false)


% % System 2 from notebook
% clear variables
% syms m1 m2 Fx k b t x1(t) x1_dot(t) x2(t) x2_dot(t) v mu_v(v) gma g
% x1_dot = diff(x1, 't')
% x2_dot = diff(x2, 't')
% T = 0.5*m1*x1_dot^2 + 0.5*m2*x2_dot^2
% V = 0.5*k*x2^2
% % v_funcs = {x1_dot, x1_dot-x2_dot}
% v_funcs = {1, 1};
% N_forces = {1, m2*g}
% % mu_1_v(v) = b*v + m2*g*sign(v)
% mu_1_v(v) = b*x1_dot + m2*g*sign(x1_dot - x2_dot);
% mu_1_2_v(v) = sign(x1_dot - x2_dot);
% % mu_1_2_v(v) = gma(1)*(tanh(gma(2)*v) - tanh(gma(3)*v)) + gma(4)*tanh(gma(5)*v) + gma(6)*v
% 
% mu_funcs = {mu_1_v, mu_1_2_v}
% coordVars = {x1, x2};
% U = [Fx; 0]
% sys = deriveEom('System_2', coordVars, T, V, N_forces, mu_funcs, v_funcs, U, false)