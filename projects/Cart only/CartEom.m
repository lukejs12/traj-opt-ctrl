function x_dot = CartEom(t, x, u, param)
b = param.b;
q1 = x(1);
q1_dot = x(2);
x_dot(1:1) = x(2:2);
x_dot(2:2) = Cart_auto(b,q1_dot,u);
x_dot = x_dot';
