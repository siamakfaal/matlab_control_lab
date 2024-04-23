clc; clear; close all;


robot = cartpole();

sim = simulator(model=robot);

x0 = robot.x0;
x0(2) = 0.1;

sol = sim.solve([0,10], x0);


sim.plot(sol);
sim.animate(sol);