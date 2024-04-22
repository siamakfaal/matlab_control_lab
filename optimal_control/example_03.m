clc; clear; close all;
addpath('../systems');

robot = cartpole();

sim = simulator(model=robot);

x0 = [0; 0.4; 0; 0];

Q = diag([4,4,1,1]);

h = [];
g = @(t,x,u) x'*(Q*x);
%g = @(t,x,u) (1 - cos(x(1)))^2;
f = @(t,x,u) robot.openloop(t,x,u);

solver = ocs(4, 1, f, h, g);

u0 = 0;
tspan = linspace(0,10,60);

options = optimoptions("fmincon", ...
                Algorithm="sqp",...
                Display= "iter-detailed",...
                MaxFunctionEvaluations=1e6,...
                MaxIterations=200);

x_bound = [-0.7 0.7; -Inf Inf; -Inf Inf; -Inf Inf];
u_bound = [-2, 2];

sol = solver.solve(tspan, x0, u0, x_bound, u_bound, options);
solver.plot(sol);
% ax = sim.animationaxes();
% xline(ax, x_bound(1,1));
% xline(ax, x_bound(1,2));
% sim.animate(sol,ax);
sim.animate(sol);
