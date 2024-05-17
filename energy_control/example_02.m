clc; clear; close all;
addpath('../systems/src');

m = [1; 1]; % [cart mass, pole mass]
l = 1;      % Pole lenght
I = 1;      % Pole inertia
c = 0.5;    % Pole center of mass
g = 9.81;

robot = cartpole(m=m,l=l,I=I,c=c,g=g);

xd = zeros(4,1);

Ed = robot.energy([], xd);

ks = [2, 1];
k = 1;
epsilon = 0.4;
w = 0.5;

A = zeros(4); A(1:2,3:4) = eye(2); 
A(3:4,2) = [c^2*g*m(2)^2; c*m(2)*(g*m(1) + g*m(2))]/(m(1)*m(2)*c^2 + m(1)*I + m(2)*I);
B = [0; 0; m(2)*c^2 + I; c*m(2)]/(m(1)*m(2)*c^2 + m(1)*I + m(2)*I);

K = lqr(A,B,diag([20,5,1,1]),1);

us = @(x) -K*x;
ue = @(x) -k*(robot.energy([],x) - Ed)*x(3);
in_omega = @(x) 1 - cos(x(1)) + x(2)^2 <= epsilon;


sim = simulator(model=robot);

x0 = robot.x0; x0(2) = 2;

sol = sim.solve(0:0.1:20, x0, @(t,x)hybrid_control(x, w, us, ue, in_omega));

sim.plot(sol);
sim.animate(sol);

ax = sim.make_plot_axes;
E = sim.eval(@robot.energy, sol);
plot(ax, sol.t, E);
sim.set_legend_properties(xlabel(ax, "$t$"), ylabel(ax, "$E(t)$"));

function u = hybrid_control(x, w, us, ue, in_omega)
    if in_omega(x)
        u = us(x);
    else
        u = ue(x);
    end
    u = max(min(u, w),-w);
end


