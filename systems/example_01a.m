clc; clear; close all;
addpath src/

robot = satellite2d();
sim = simulator(model=robot);
x0 = [0; 1];
sol = sim.solve(0:0.1:5,x0);

sim.plot(sol);
sim.animate(sol);
