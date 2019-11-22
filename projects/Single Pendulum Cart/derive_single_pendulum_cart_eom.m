% clear variables;
syms m1 b1 m2 I2 c2 l2 b2 q1(t) q2(t) g u
sysName = 'SinglePendulumCart';
coordVars = {q1, q2};

x2 = q1 + c2*sin(q2); y2 = -c2*cos(q2);
T = .5*m1*diff(q1, 't')^2 + .5*m2*diff(x2, 't')^2 + .5*m2*diff(y2, 't')^2 ...
    + .5*I2*diff(q2, 't')^2;
V = m2*g*c2*(1-cos(q2));
D = .5*b1*diff(q1, 't')^2 + .5*b2*diff(q2, 't')^2;
L = T - V;
Q = [u; 0];
sys = deriveEom(sysName, coordVars, L, D, Q, true);