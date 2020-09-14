clc;
clear variables;
syms m1 m2 I2 c2 l2 q1(t) q2(t) g u
sysName = 'SinglePendulumCart';
coordVars = {q1, q2};

q1_dot = diff(q1, 't');
q2_dot = diff(q2, 't');
x2 = q1 + c2*sin(q2); y2 = -c2*cos(q2);
T = .5*m1*diff(q1, 't')^2 + .5*m2*diff(x2, 't')^2 + .5*m2*diff(y2, 't')^2 ...
    + .5*I2*diff(q2, 't')^2;
V = m2*g*c2*(1-cos(q2));
% D = .5*b1*diff(q1, 't')^2 + .5*b2*diff(q2, 't')^2;
L = T - V;
mu = @(qd, gamma) gamma(1)*(tanh(gamma(2)*qd - tanh(gamma(3)*qd))) + ...
    gamma(4)*tanh(gamma(5)*qd) + gamma(6)*qd;
gamma1 = sym('g1_', [1, 6]);
gamma2 = sym('g2_', [1, 6]);
Q = [-mu(q1_dot, gamma1); -mu(q2_dot, gamma2)];
D = 0;
U = [u; 0];
sys = deriveEom(sysName, coordVars, L, D, U, Q, true);