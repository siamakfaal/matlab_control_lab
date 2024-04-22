clc; clear; close all;
addpath('../systems');

robot = pendulum();

sim = simulator(model=robot);

x0 = [pi; 0];

Q = diag([4,1]);

h = [];
%g = @(t,x,u) x'*(Q*x);
g = @(t,x,u) (1 - cos(x(1)))^2;
f = @(t,x,u) robot.openloop(t,x,u);

solver = ocs(2, 1, f, h, g);

u0 = 0;
tspan = linspace(0,7,70);

options = optimoptions("fmincon", ...
                Algorithm="active-set",...
                Display= "iter-detailed",...
                MaxFunctionEvaluations=1e6,...
                MaxIterations=200);

x_bound = [];
u_bound = [-2, 4];

sol = solver.solve(tspan, x0, u0, x_bound, u_bound, options);
solver.plot(sol);
sim.animate(sol);
