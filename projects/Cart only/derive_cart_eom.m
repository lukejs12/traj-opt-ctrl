% clear variables;
syms m b q1(t) u
sysName = 'Cart';
coordVars = {q1};

T = .5*b*diff(q1, 't')^2;
V = 0;
D = .5*b*diff(q1, 't')^2;
L = T - V;
Q = [u];
sys = deriveEom(sysName, coordVars, L, D, Q, true);