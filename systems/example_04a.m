clc; clear; close all;
addpath("src/");

model = rrarm();
sim = simulator(model=model);

u_fl = @(t,x,v) model.M(t,x)*v(t,x) + model.h(t,x);

qd = [pi/4; pi/4];

mu = [-1 -2 -3 -4];

A = zeros(4); A(1:2,3:end) = eye(2);
B = [zeros(2,2);eye(2)];

K = place(A,B,mu);

v = @(t,x) K*([qd; 0;0] - x);


x0 = model.x0;

sol = sim.solve([0,5],x0, @(t,x) u_fl(t,x,v));

sim.plot(sol);
sim.animate(sol);