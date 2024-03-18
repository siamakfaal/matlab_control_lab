clc; clear; 
%close all;

f = @(t, x)[x(2); -3*x(1)+x(2)];

xticks = 0;
yticks = 0;

phaseplane = portrait([-2, 0.4, 2 ; -2, 0.4, 2], ...
    xticks=xticks, yticks=yticks, ...
    trajectory_color=[0.8, 0.8, 0.8]);

x0 = phaseplane.icgrid('lhr');


ax = phaseplane.buildaxes(figure(Position=[118, 314, 650, 250]));

phaseplane.draw(f,x0,[0,20],ax);

plot(ax, xticks, zeros(size(xticks)), 'o',...
    Color=lines(1), MarkerFaceColor=lines(1), MarkerSize=5);

set(ax,XTickLabel=xticklabels)