clc; clear; close all;

model = pendulum(b = 0, g=1);
sim = simulator(model=model);

sol = sim.solve([0,10],[1;0], @(t,x) -3*x(1)-x(2) );

sim.plot(sol);
sim.animate(sol);