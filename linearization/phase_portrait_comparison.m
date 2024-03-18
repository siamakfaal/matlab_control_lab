clc; clear; close all;
addpath('../phase_portraits');

% Zoom level
zoom = 2;


% Nonlinear system
f = @(t,x)[x(1)*cos(x(2)) + x(2)^2; x(1)*(x(1)+1) + x(1)*sin(x(2)) + x(2)];

% Linear Approximation at the origin
x = sym('x%d',[2,1],'real');
A = double(subs(jacobian(f(0,x),x),x,[0;0]));
f_ = @(t,x) A*x;

% Phase Portraits
phaseplane = portrait(zoom*[-1, 0.1 ,1 ; -1, 0.1, 1]);
x0 = phaseplane.icgrid();
xeq = phaseplane.findequilibria(f,1,1e-5);

ax1 = phaseplane.trajectories(f,x0,[0,10]);
ax2 = phaseplane.trajectories(f_,x0,[0,10]);

title(ax1,'Nonlinear System', ...
    Interpreter='latex',FontSize=22)

title(ax2,'Linear Approximation', ...
    Interpreter='latex',FontSize=22)

plot(ax1, xeq(:,1), xeq(:,2), ...
    Marker="o", MarkerEdgeColor="None", ...
    MarkerFaceColor=[0 1]*lines(2),...
    MarkerSize=10,...
    LineStyle="None");

plot(ax2, 0, 0, ...
    Marker="o", MarkerEdgeColor="None", ...
    MarkerFaceColor=[0 1]*lines(2),...
    MarkerSize=10,...
    LineStyle="None");
