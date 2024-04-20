clc; clear; close all;


robot = satellite2d();

sim = simulator(model=robot);

x0 = [0; 1];

sol = sim.solve(0:0.01:60, x0);


sim.plot(sol);
tic
sim.animate(sol);
toc