clc; clear all; close all; %#ok<CLALL>

q = sym('q%d',[2,1],'real');
dq = sym('dq%d',[2,1],'real');
u = sym('u','real');
syms m1 m2 l1 l2 g 'positive'

m = [m1, m2, m2];

p = sym(zeros(3,2));
p(1,:) = [-l1*sin(q(1)), l1*cos(q(1))];
p(2,:) = p(1,:) + [l2*cos(q(1)+q(2)), l2*sin(q(1)+q(2))];
p(3,:) = p(1,:) - [l2*cos(q(1)+q(2)), l2*sin(q(1)+q(2))];


V = 0; U = 0;
for i = 1:3
    dp = jacobian(p(i,:)',q)*dq;
    V = V + m(i)*(dp'*dp)/2;
    U = U + m(i)*g*p(i,2);
end

dV = gradient(V,dq);

M = simplify(jacobian(dV,dq));
Phi = simplify(jacobian(dV,q)*dq - gradient(V,q) + gradient(U,q));
T = [0; 1];


syms a b c 'positive'

M = subs(M, [m1*l1^2 + 2*m2*l1^2, m2*l2^2], [a, b]);
Phi = subs(Phi, l1*(m1 + 2*m2), a/l1);


x = [q; dq];
xeq = zeros(4,1);
ueq = 0;

f = [x(3:4); M\(T*u - Phi)];
A = simplify(subs(jacobian(f,x),[x; u],[xeq; ueq]));
B = simplify(subs(jacobian(f,u),[x; u],[xeq; ueq]));

Ctrb = simplify([B A*B A^2*B A^3*B]);

p = fliplr(poly(-4:-1));
phi = zeros(size(A));
for k = 1:length(p)
    phi = phi + p(k)*A^(k - 1);
end


K = [0 0 0 1]*(Ctrb\phi);


