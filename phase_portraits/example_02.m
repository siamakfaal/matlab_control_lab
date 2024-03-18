clc; clear; close all;

xi1 = -1; xi2 = -3;

A = [0 1; 1 2];
B = eye(2);

k1 = 1 - (xi1 + xi2)/2;
k2 = 1 + sqrt((xi1 - xi2)^2 - 4);

K = [k1, k2; k2 k1];

f = @(t,x) (A - B*K)*x;

xticks = 0;
yticks = 0;


phaseplane = portrait([-2, 0.5, 2 ; -2, 0.5, 2], ...
    xticks=xticks, yticks=yticks);

x0 = phaseplane.icgrid();

ax = phaseplane.draw(f,x0,[0,10]);

plot(ax, xticks, zeros(size(xticks)), 'o',...
    MarkerFaceColor=[0 0 0 1]*lines(4),...
    MarkerSize=10);
