clc; clear; close all;
addpath src/

b = 0.1;
g = 9.81;

model = pendulum(b=b, g=g);
sim = simulator(model=model);

x0 = [3/4*pi; 0];

ctrl = @(t,x) -[25, 5]*(x - [0.5;0]);

sol = sim.solve(0:0.05:5,x0,ctrl);

sim.plot(sol);
sim.animate(sol);

e = sim.eval(@model.energy, sol);

ax = sim.make_plot_axes;
plot(ax,sol.t,e, LineWidth=1);
sim.set_legend_properties(xlabel(ax,"$t$"), ylabel(ax,"$E$"));
