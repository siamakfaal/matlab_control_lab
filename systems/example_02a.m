clc; clear; close all;

model = pendulum(b=0.2);
sim = simulator(model=model);

x0 = [3/4*pi; 0];

sol = sim.solve(0:0.1:10,x0);

sim.plot(sol);
sim.animate(sol);

e = model.eval(@model.energy, sol);

ax = sim.make_plot_axes;
plot(ax,sol.t,e);
sim.set_legend_properties(xlabel(ax,"$t$"), ylabel(ax,"$E$"));
