clc; clear all; close all; %#ok<CLALL>

syms x theta dx dtheta d2x d2theta u 'real'
syms m1 m2 I c g 'positive'

q = [x; theta];
dq = [dx; dtheta];

p = [x - c*sin(theta); c*cos(theta)];
dp = jacobian(p,[x,theta])*[dx; dtheta];

V = (m1*dx^2 + m2*(dp'*dp) + I*dtheta^2)/2;
U = m2*g*p(2);

dV = gradient(V,dq);

M = simplify(jacobian(dV,dq));
b = simplify(jacobian(dV,q)*dq - gradient(V,q));
gr = simplify(gradient(U,q));

% Equations of motion are: M*d2q + b + gr = [u; 0]

f = [dq; M\([u; 0] - b - gr)];

A = subs(jacobian(f,[q;dq]), [q;dq;u],[0;0;0;0;0]);
B = subs(jacobian(f,u), [q;dq;u],[0;0;0;0;0]);