clc; clear; close all

g = 9.81;  % The gravitational acceleration [m/s^2]
l = 0.2;  % Distance from the center of mass to each rotor [m]
m = 0.5;  % Total mass of the quadrotor [kg]
I = [1.24, 1.24, 2.48];  % Mass moment of inertia [kg m^2]
mu = 3.0;  % Maximum thrust of each rotor [N]
sigma = 0.01;  % The proportionality constant relating thrust to torque [m]

robot = quadrotor(g, l, m, diag(I), mu, sigma);

%% Simple LQR Design on Error Dynamics
% e = zd - z => z = zd - e
% v = ud - u => u = ud - v

e = sym('e%d', [12,1],'real');
v = sym('v%d', [4,1], 'real');

zd = zeros(12,1); 
ud = ones(4,1)*m*g/4;

de = -robot.state_space(zd - e, ud - v);

A = double(subs(jacobian(de,e),[e;v],zeros(12+4,1)));
B = double(subs(jacobian(de,v),[e;v],zeros(12+4,1)));

K = lqr(A, B, eye(12), eye(4));

u = @(t,z,zd) ud + K*(zd - z);

%% Simulation
z0 = zeros(12,1);
zd = @(t) [cos(t); sin(t); 2; zeros(3,1); zeros(3,1); zeros(3,1)];
tspan = [0, 20];

[t,z] = robot.solve(tspan, z0, @(t,z)u(t,z,zd(t)));

robot.plot(t,z)
robot.animate(t,z)