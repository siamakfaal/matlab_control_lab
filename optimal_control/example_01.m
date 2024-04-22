clc; clear; close all;

robot = satellite2d();

sim = simulator(model=robot);

x0 = [0; pi];
xd = [0; 0];

h = @(t,x) norm(x-xd);
g = @(t,x,u) norm(x - xd);
f = @(t,x,u) robot.openloop(t,x,u);

solver = ocs(2, 1, f, h, g);

u0 = 0;
tspan = linspace(0,5,50);

options = optimoptions("fmincon", ...
                Algorithm="sqp",...
                Display= "iter-detailed",...
                MaxFunctionEvaluations=1e6,...
                MaxIterations=200);

x_bound = [];
u_bound = [-2, 2];

sol = solver.solve(tspan, x0, u0, x_bound, u_bound, options);
solver.plot(sol);
sim.animate(sol);
