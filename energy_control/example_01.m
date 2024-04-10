clc; clear; close all;

addpath('../systems');

model = pendulum(b=0, g=1);

ks = [2, 1];
k = 2;
epsilon = 0.2;
w = 0.5;

x0 = [3; 0];

us = @(x) -ks*x;
ue = @(x) -k*(model.energy(x) - 1)*sign(x(2));
in_omega = @(x) 1 - cos(x(1)) + x(2)^2 <= epsilon;


sim = simulator(model=model);
sol = sim.solve(0:0.05:40, x0, @(t,x) hybrid_control(x,w,us,ue,in_omega));
ax = sim.plot(sol);
sim.animate(sol);


function u = hybrid_control(x, w, us, ue, in_omega)
    if in_omega(x)
        u = us(x);
    else
        u = ue(x);
    end
    u = max(min(u, w),-w);
end