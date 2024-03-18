clc; clear; close all;

f = @(t, x)[x(1)+x(2)+x(1)^2+x(2)^2; x(1)-x(2)-x(1)^2+x(2)^2];

phaseplane = portrait([-2.5, 0.2 ,1 ; -2.5, 0.2, 1]);
x0 = phaseplane.icgrid();
phaseplane.draw(f,x0,[0,10]);
