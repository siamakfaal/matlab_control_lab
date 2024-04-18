clc; clear; close all;

A = [2 -1; 0 0.5];

N = 100;

q = linspace(0, 2*pi, N+1)'; q(end) = [];
x = [cos(q), sin(q)];

[v,d] = eig(A);

y = x*A';

q_eig = atan2(v(2,:)',v(1,:)');
q_eig = [q_eig; pi+q_eig];

x_eig = [cos(q_eig), sin(q_eig)];
y_eig = x_eig*A';

ax = axes(NextPlot="add",DataAspectRatio=[1,1,1], XGrid="on", YGrid="on",...
    Box="on", TickLabelInterpreter="latex", FontSize=20,...
    Xlim=[-2.4,2.4], Ylim=[-1.1,1.1]);

colors = lines(1);

color_high = [0.4940    0.1840    0.5560];

scatter(ax, x(:,1), x(:,2), 50, colors, "filled", MarkerFaceAlpha=0.7);
scatter(ax, y(:,1), y(:,2), 50, colors, "filled", MarkerFaceAlpha=1);

scatter(ax, x_eig(:,1), x_eig(:,2), 150, color_high, "filled", MarkerFaceAlpha=0.7);
scatter(ax, y_eig(:,1), y_eig(:,2), 150, color_high, "filled", MarkerFaceAlpha=1);


quiver(ax,x_eig(:,1),x_eig(:,2),y_eig(:,1)-x_eig(:,1), y_eig(:,2)-x_eig(:,2),0,Color=[0 0 0],LineWidth=2)

quiver(ax,[0;0],[0;0],v(1,:)', v(2,:)',0.4,"LineWidth",2,...
    MaxHeadSize=20);

