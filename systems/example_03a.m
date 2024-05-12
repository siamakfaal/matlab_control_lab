clc; clear; close all;
addpath src/

robot = cartpole();
sim = simulator(model=robot);

x0 = robot.x0;
x0(2) = 0.1;

sol = sim.solve([0,10], x0);
sim.plot(sol);
sim.animate(sol);

total_energy = sim.eval(@robot.energy,sol);
ax = sim.make_plot_axes;
plot(ax,sol.t,total_energy,LineWidth=1);
sim.set_legend_properties(xlabel(ax,"$t$"), ylabel(ax,"$E$"));
