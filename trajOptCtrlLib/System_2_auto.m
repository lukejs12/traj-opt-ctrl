function q_ddot_symvar = System_2_auto(b,g,g1_1,g1_2,g1_3,g1_4,g1_5,g1_6,k,m1,m2,u,x1_dot,x2,x2_dot)
%SYSTEM_2_AUTO
%    Q_DDOT_SYMVAR = SYSTEM_2_AUTO(B,G,G1_1,G1_2,G1_3,G1_4,G1_5,G1_6,K,M1,M2,U,X1_DOT,X2,X2_DOT)

%    This function was generated by the Symbolic Math Toolbox version 6.3.
%    25-Feb-2020 16:15:39

t2 = x1_dot-x2_dot;
t3 = g1_3.*t2;
t4 = tanh(t3);
t5 = g1_2.*x2_dot;
t6 = t4+t5-g1_2.*x1_dot;
t7 = tanh(t6);
t8 = g1_5.*t2;
t9 = tanh(t8);
q_ddot_symvar = [(u-b.*x1_dot+g.*g1_1.*m2.*t7-g.*g1_4.*m2.*t9-g.*g1_6.*m2.*x1_dot+g.*g1_6.*m2.*x2_dot)./m1;-g.*(g1_1.*t7-g1_4.*t9-g1_6.*x1_dot+g1_6.*x2_dot)-(k.*x2)./m2];