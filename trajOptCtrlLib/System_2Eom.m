function x_dot = System_2Eom(t, x, u, param)
b = param.b;
g = param.g;
g1_1 = param.g1_1;
g1_2 = param.g1_2;
g1_3 = param.g1_3;
g1_4 = param.g1_4;
g1_5 = param.g1_5;
g1_6 = param.g1_6;
k = param.k;
m1 = param.m1;
m2 = param.m2;
x1 = x(1);
x1_dot = x(3);
x2 = x(2);
x2_dot = x(4);
x_dot(1:2) = x(3:4);
x_dot(3:4) = System_2_auto(b,g,g1_1,g1_2,g1_3,g1_4,g1_5,g1_6,k,m1,m2,u,x1_dot,x2,x2_dot);
x_dot = x_dot';
