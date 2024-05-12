clc; clear; close all;

robot = satellite2d();
sim = simulator(model=robot);
x0 = [1; 1];

ctrl = @(t,x) -[3 2]*x;

sol = sim.solve(0:0.01:10, x0, ctrl);

sim.plot(sol);
sim.animate(sol);
